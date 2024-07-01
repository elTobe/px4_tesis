import rclpy
import math
import numpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus
from std_msgs.msg import Empty
from geometry_msgs.msg import PoseStamped, Twist, Pose

from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

# Currently only position trayectories, and passtrough velocities and setpoints are meant to be supported, might change in a future
class PX4Driver(Node):

    def __init__(self):
        super().__init__("px4_driver")
        self.get_logger().info("Starting PX4 driver node ...")

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Initialize variables
        self.takeoff_height = -1
        self.node_rate = 10
        self.node_dt = 1/self.node_rate
        self.current_setpoint = TrajectorySetpoint()
        self.takeoff_position = VehicleLocalPosition()
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()

        # Flags
        self.is_taking_off = False
        self.is_in_trajectory = False

        # Create publishers
        self.drone_pose_publisher = self.create_publisher(PoseStamped, "/px4_driver/drone_pose", 10)
        self.takeoff_pose_publisher = self.create_publisher(PoseStamped, "/px4_driver/takeoff_pose", 10)
        self.offboard_control_mode_publisher = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", qos_profile)
        self.vehicle_command_publisher = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", qos_profile)

        # Create subscribers
        self.vehicle_local_position_subscriber = self.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position", self.vehicle_local_position_update, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(VehicleStatus, "/fmu/out/vehicle_status", self.vehicle_status_update, qos_profile)
        self.takeoff_subscriber = self.create_subscription(Empty, "/px4_driver/takeoff", self.take_off, 10)
        self.land_subscriber = self.create_subscription(Empty, "/px4_driver/land", self.land, 10)
        self.velocity_subscriber = self.create_subscription(Twist, "/px4_driver/cmd_vel", self.cmd_vel, 10)
        self.position_subscriber = self.create_subscription(Pose, "/px4_driver/cmd_pos", self.cmd_pos, 10)

        # Frame Broadcaster for Rviz
        self.tf_broadcaster = TransformBroadcaster(self)

        # Create a timer to publish hearbeat and setpoints (must be >= 2 Hz)
        self.heartbeat_timer = self.create_timer( self.node_dt, self.heartbeat)
    
    # Returns quaternion [x, y, z, w] from Euler angles
    def quaternion_from_euler(self, roll=0, pitch=0, yaw=0):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = [0] * 4
        q[0] = cy * cp * cr + sy * sp * sr
        q[1] = cy * cp * sr - sy * sp * cr
        q[2] = sy * cp * sr + cy * sp * cr
        q[3] = sy * cp * cr - cy * sp * sr

        return q
    
    # Returns euler angles from quaternion
    def euler_from_quaternion(self, quaternion):
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = numpy.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = numpy.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = numpy.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    # Update local position when a new message is published
    def vehicle_local_position_update(self, msg):
        self.vehicle_local_position = msg

        # Publish drone pose
        drone_pose = PoseStamped()
        drone_pose.header.frame_id = "world"
        drone_pose.pose.position.x = msg.x
        drone_pose.pose.position.y = msg.y
        drone_pose.pose.position.z = msg.z
        q = self.quaternion_from_euler(yaw=msg.heading)
        drone_pose.pose.orientation.x = q[0]
        drone_pose.pose.orientation.y = q[1]
        drone_pose.pose.orientation.z = q[2]
        drone_pose.pose.orientation.w = q[3]
        drone_pose.header.stamp = self.get_clock().now().to_msg()
        self.drone_pose_publisher.publish(drone_pose)

        # Publish takeoff position
        takeoff_pose = PoseStamped()
        takeoff_pose.header.frame_id = "world"
        takeoff_pose.pose.position.x = float(self.takeoff_position.x)
        takeoff_pose.pose.position.y = float(self.takeoff_position.y)
        takeoff_pose.pose.position.z = float(self.takeoff_position.z)
        q = self.quaternion_from_euler(yaw=self.takeoff_position.heading)
        takeoff_pose.pose.orientation.x = q[0]
        takeoff_pose.pose.orientation.y = q[1]
        takeoff_pose.pose.orientation.z = q[2]
        takeoff_pose.pose.orientation.w = q[3]
        takeoff_pose.header.stamp = self.get_clock().now().to_msg()
        self.takeoff_pose_publisher.publish(takeoff_pose)

        # Transform takeoff position frame for rviz
        t = TransformStamped()
        t.child_frame_id = "local_home"
        t.header.frame_id = "world"
        t.transform.translation.x = takeoff_pose.pose.position.x
        t.transform.translation.y = takeoff_pose.pose.position.y
        t.transform.translation.z = takeoff_pose.pose.position.z
        t.header.stamp = self.get_clock().now().to_msg()
        self.tf_broadcaster.sendTransform(t)

    # Update status when a new message is published
    def vehicle_status_update(self, msg):
        self.vehicle_status = msg

    # Publish a vehicle command with current timestamp, and a variable amount of parameters. Parameters not passed are default to a 0 value
    # Command value and parameters are specified in the message definition of px4_msgs/msg/VehicleCommand
    def publish_vehicle_command(self, command, **params):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    # Arm vehicle
    def arm(self):
        # TODO : check if arming is possible or verify it armed
        self.get_logger().info('Arming vehicle')
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)

    # Disarm vehicle
    def disarm(self):
        self.get_logger().info('Disarming vehicle')
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)

    # Change vehicle to offboard mode (only possible after sending at least 1 second of constant heartbeat signal)
    def engage_offboard_mode(self):
        #TODO : check if we are in offboard
        self.get_logger().info("Switching to offboard mode")
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)

    # Land the vehicle
    def land(self, msg=None):
        self.get_logger().info("Landing vehicle")
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)

    # Takeoff vehicle
    def take_off(self, msg=None):
        if self.vehicle_status.takeoff_time != 0:
            self.get_logger().info("Vehicle already flying, not taking off")
            return

        # TODO: check for valid local position
        self.get_logger().info("Taking Off")
        self.engage_offboard_mode()
        self.arm()

        # Save takeoff position
        self.takeoff_position.x = self.vehicle_local_position.x
        self.takeoff_position.y = self.vehicle_local_position.y
        self.takeoff_position.z = self.vehicle_local_position.z
        self.takeoff_position.heading = self.vehicle_local_position.heading

        # Update target setpoint to be take off +  the height
        self.current_setpoint = TrajectorySetpoint()
        self.current_setpoint.position[0] = self.takeoff_position.x
        self.current_setpoint.position[1] = self.takeoff_position.y
        self.current_setpoint.position[2] = self.takeoff_position.z + self.takeoff_height
        self.current_setpoint.yaw = self.takeoff_position.heading

    # Update setpoint to perform velocity control when Twist received
    def cmd_vel(self, msg):
        # TODO: normalize to -1 .. 1
        self.current_setpoint.velocity[0] = msg.linear.x
        self.current_setpoint.velocity[1] = msg.linear.y
        self.current_setpoint.velocity[2] = msg.linear.z
        self.current_setpoint.yawspeed = msg.angular.z

    # Update setpoint to perform position control when Pose received
    def cmd_pos(self, msg):
        # Change position setpoint
        self.current_setpoint.position[0] = msg.position.x + self.takeoff_position.x
        self.current_setpoint.position[1] = msg.position.y + self.takeoff_position.y
        self.current_setpoint.position[2] = msg.position.z + self.takeoff_position.z

        # Only change yaw if orientation is not identity quaternion
        #if msg.orientation.w != 1:
        #    q = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        #    _, _, yaw = self.euler_from_quaternion(q)
        #    self.current_setpoint.yaw = yaw + self.takeoff_position.heading

    # Send hearbeat and current setpoint (this signal must be sent constantly when in offboard mode, in a frecuency >= 2 Hz)
    def heartbeat(self):

        # Send type of control to perform
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        if numpy.linalg.norm(self.current_setpoint.velocity) != 0 or self.current_setpoint.yawspeed != 0:
            msg.velocity = True
            msg.position = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

        # Publish setpoint, update position value to make drone hover after effects of velocity control
        #if msg.velocity:
        #    self.current_setpoint.position[0] = self.vehicle_local_position.x
        #    self.current_setpoint.position[1] = self.vehicle_local_position.y
        #    self.current_setpoint.position[2] = self.vehicle_local_position.z
        #    self.current_setpoint.yaw = self.vehicle_local_position.heading
        self.current_setpoint.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(self.current_setpoint)

# Definition of main function to create and run the node
def main(args=None):
    rclpy.init(args=args)
    px4_driver = PX4Driver()
    
    rclpy.spin(px4_driver)

    px4_driver.destroy_node()
    rclpy.shutdown()

# Call main function
if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)