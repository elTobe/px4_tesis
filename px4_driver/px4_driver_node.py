import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus

# Currently only position trayectories, and passtrough velocities and setpoints are meant to be supported, might change in a future
class PX4Driver(Node):

    def __init__(self) -> None:
        super().__init__('px4_driver')

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create publishers
        self.offboard_control_mode_publisher = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Create subscribers
        self.vehicle_local_position_subscriber = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)

        # Initialize user variables
        self.takeoff_height = -1.0
        self.node_rate = 10
        self.takeoff_secs = 5
        self.offboard_secs = 2

        # Initialize system variables
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.offboard_setpoint_counter = 0
        self.setpoint_index = -1
        self.setpoints_for_offboard = int(self.offboard_secs*self.node_rate) + 1
        self.setpoints_for_takeoff = int(self.takeoff_secs*self.node_rate) + 1
        self.setpoints_offset = self.setpoints_for_offboard + self.setpoints_for_takeoff

        # Elements for setpoint trajectory are :
        # x_ned (in meters relative to origin)
        # y_ned (in meters relative to origin)
        # z_ned (in meters relative to origin and always negative)
        # yaw (in radians from -pi to pi)
        # secs since takeoff completed (always positive and continuously growing between each setpoint)
        self.current_position_trayectory = [[1, 0, -1, 0, 10],
                                            [1, 0, -2, 0, 20],
                                            [0, 0, -2, 0, 30],
                                            [0, 0, -1, 0, 40]]

        # Create a timer to publish hearbeat and setpoints (must be >= 2 Hz)
        self.timer = self.create_timer(1/self.node_rate, self.timer_callback)

    # Update local position when a new message is published
    def vehicle_local_position_callback(self, msg):
        self.vehicle_local_position = msg

    # Update status when a new message is published
    def vehicle_status_callback(self, msg):
        self.vehicle_status = msg

    # Arm vehicle
    def arm(self):
        self.get_logger().info('Arming vehicle')
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)

    # Disarm vehicle
    def disarm(self):
        self.get_logger().info('Disarming vehicle')
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)

    # Change vehicle to offboard mode (only possible after sending at least 1 second of constant heartbeat signal)
    def engage_offboard_mode(self):
        self.get_logger().info("Switching to offboard mode")
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)

    # Land the vehicle
    def land(self):
        self.get_logger().info("Landing vehicle")
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)

    # Send hearbeat signal (this signal must be sent constantly when in offboard mode, in a frecuency >= 2 Hz)
    # The message sent here says the type of control we are using in the trayectory setpoint (position, velocity, position & velocity, etc)
    def publish_offboard_control_heartbeat_signal(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    # Publish a trayectory setpoint, can be postition, vel, acc or any combination of those. This is defined by the heartbeat signal.
    def publish_position_setpoint(self, x, y, z, yaw ):
        msg = TrajectorySetpoint()
        msg.position = [float(x), float(y), float(z)]
        msg.yaw = float(yaw)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.get_logger().info(f"Publishing x={float(x)}, y={float(y)}, z={float(z)}, yaw={float(yaw)}")
        self.trajectory_setpoint_publisher.publish(msg)

    # Publish a vehicle command with current timestamp, and a variable amount of parameters. Parameters not passed are default to a 0 value
    # Command value and parameters are specified in the message definition of px4_msgs/msg/VehicleCommand
    def publish_vehicle_command(self, command, **params) -> None:
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

    # Loop of the node, publish hearbeat and setpoints
    def timer_callback(self) -> None:

        # Send hearbeat, current setpoint and increase offboar setpoint counter
        self.publish_offboard_control_heartbeat_signal()
        if self.setpoint_index < 0:
            self.publish_position_setpoint(0, 0, -self.takeoff_height, 0)
        else:
            print(f"current index : {self.setpoint_index}")
            temp_setpoint = self.current_position_trayectory[self.setpoint_index]
            self.publish_position_setpoint(temp_setpoint[0], # x
                                           temp_setpoint[1], # y
                                           temp_setpoint[2], # z
                                           temp_setpoint[3]) # yaw
            if self.offboard_setpoint_counter > temp_setpoint[4]*self.node_rate + self.setpoints_offset:
                if self.setpoint_index == len(self.current_position_trayectory)-1:
                    self.land()
                    exit()
                else:
                    self.setpoint_index +=1
        self.offboard_setpoint_counter +=1
        
        # Change to offboard when 1 second has passed
        if self.offboard_setpoint_counter == self.setpoints_for_offboard:
            self.engage_offboard_mode()
            self.arm()

        # Start trayectory when takeoff is completed
        if self.offboard_setpoint_counter == self.setpoints_offset:
            self.setpoint_index += 1

# Definition of main function to create and run the node
def main(args=None) -> None:

    print('Starting PX4 driver ...')
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
