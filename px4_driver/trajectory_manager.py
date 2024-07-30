import rclpy
import time
import math
import numpy
from rclpy.node import Node
from threading import Thread
from nav_msgs.msg import Path
from std_msgs.msg import Float32, Empty, Int32
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster, TransformListener, Buffer

class TrajectoryManager(Node):
    def __init__(self):
        super().__init__("trajectory_manager")
        self.get_logger().info("Starting trajectory manager node ...")

        # Initialize Variables
        self.trajectory_velocity = 0.16
        self.takeoff_time = 10
        self.drone_pose = PoseStamped()
        self.current_setpoint_index = 0
        self.current_trajectory = None
        self.orig_path = None
        self.trajectory_transform = TransformStamped()
        self.is_in_trajectory = False
        self.node_rate = 10
        self.node_dt = 1/self.node_rate

        # Create Publishers
        self.trajectory_progress_publisher = self.create_publisher(Int32, "/trajectory_manager/current_index", 10)
        self.path_publisher = self.create_publisher(Path, "/trajectory_manager/current_path", 10)
        self.orig_path_publisher = self.create_publisher(Path, "/trajectory_manager/orig_path", 10)
        self.takeoff_publisher = self.create_publisher(Empty, "/px4_driver/takeoff", 10)
        self.land_publisher = self.create_publisher(Empty, "/px4_driver/land", 10)
        self.pos_publisher = self.create_publisher(PoseStamped, "/px4_driver/cmd_pos", 10)

        # Create Subscribers
        self.drone_pose_subscriber = self.create_subscription(PoseStamped, "/px4_driver/drone_pose", self.drone_pose_update,10)
        self.set_trajectory_subscriber = self.create_subscription(Path, "/trajectory_manager/set_path", self.set_trajectory, 10)
        self.set_velocity_subscriber = self.create_subscription(Float32, "/trajectory_manager/set_velocity", self.set_velocity, 10)
        self.start_trajectory_subscriber = self.create_subscription(Empty, "/trajectory_manager/start", self.start_trajectory, 10)
        self.stop_trajectory_subscriber = self.create_subscription(Empty, "/trajectory_manager/stop", self.stop_trajectory, 10)

        # Frame Broadcasters and Listeners for Rviz2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Timer for publishing frames, current path, current index
        self.details_heartbeat = self.create_timer(self.node_dt, self.details_heartbeat)

    #Convert a quaternion into a full three-dimensional rotation matrix.
    def transform_pose_st(self, pose_st, transform):
        vector = numpy.array([pose_st.pose.position.x, pose_st.pose.position.y, pose_st.pose.position.z])

        # Apply rotation
        q0, q1, q2, q3 = transform.transform.rotation.w, transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z
        r00 = 2 * (q0 * q0 + q1 * q1) - 1
        r01 = 2 * (q1 * q2 - q0 * q3)
        r02 = 2 * (q1 * q3 + q0 * q2)   
        r10 = 2 * (q1 * q2 + q0 * q3)
        r11 = 2 * (q0 * q0 + q2 * q2) - 1
        r12 = 2 * (q2 * q3 - q0 * q1)
        r20 = 2 * (q1 * q3 - q0 * q2)
        r21 = 2 * (q2 * q3 + q0 * q1)
        r22 = 2 * (q0 * q0 + q3 * q3) - 1
        rot_matrix = numpy.array([[r00, r01, r02],
                            [r10, r11, r12],
                            [r20, r21, r22]])
        vector_rotated = numpy.dot(rot_matrix, vector)
        
        # Apply translation
        traslation_vec = numpy.array([transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z])
        result_vector = vector_rotated + traslation_vec

        # Update values assuming pose_st is passed by reference
        return result_vector

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
    
    # Returns quaternion [w, x, y, z] from Euler angles
    def quaternion_from_euler(self, roll=0, pitch=0, yaw=0):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = numpy.array([0.0, 0.0, 0.0, 0.0])
        q[0] = cy * cp * cr + sy * sp * sr
        q[1] = cy * cp * sr - sy * sp * cr
        q[2] = sy * cp * sr + cy * sp * cr
        q[3] = sy * cp * cr - cy * sp * sr

        #q = q / numpy.linalg.norm(q)
        return q

    # Update current drone pose
    def drone_pose_update(self, msg):
        self.drone_pose = msg

    # Send tfs, path
    def details_heartbeat(self):
        if self.current_trajectory is not None:
            self.trajectory_transform.header.stamp = self.get_clock().now().to_msg()
            self.tf_broadcaster.sendTransform(self.trajectory_transform)
            self.path_publisher.publish(self.current_trajectory)
            self.orig_path_publisher.publish(self.orig_path)

    # Set velocity for trayectory following
    def set_velocity(self, msg):
        self.velocity = msg.data
        self.get_logger().info(f"Velocity has been set to {self.velocity}")

    # Set ned trajectory
    def set_trajectory(self, msg):
        self.orig_path = msg
        new_path = Path()
        new_path.header.frame_id = "ned_earth"

        # Obtain transform between reference frames    
        transform = TransformStamped()
        yaw_transform = 0
        if msg.header.frame_id == "":
            transform.header.frame_id = "ned_earth"
            self.get_logger().info("Not reference frame specified, defaulting to ned_earth")
        else:
            try:
                transform = self.tf_buffer.lookup_transform("ned_earth", msg.header.frame_id, rclpy.time.Time())
                _, _, yaw_transform = self.euler_from_quaternion(transform.transform.rotation)
            except:
                self.get_logger().info("Error getting transform for reference frame, trayectory not set")
                return
        transform.child_frame_id = "path_origin"
        self.trajectory_transform = transform
        
        # Convert point in trajectory from its reference frame to ned_earth
        for i, pose_st in enumerate(msg.poses):
            new_pose = PoseStamped()
            vector = self.transform_pose_st(pose_st, transform)
            new_pose.pose.position.x = vector[0]
            new_pose.pose.position.y = vector[1]
            new_pose.pose.position.z = vector[2]

            _, _, yaw_path = self.euler_from_quaternion(pose_st.pose.orientation)
            quat = self.quaternion_from_euler(roll=0.0, pitch=0.0, yaw=(yaw_path + yaw_transform))
            new_pose.pose.orientation.w = quat[0]
            new_pose.pose.orientation.x = quat[1]
            new_pose.pose.orientation.y = quat[2]
            new_pose.pose.orientation.z = quat[3]

            new_path.poses.append(new_pose)

        self.current_trajectory = new_path
        self.get_logger().info("Trayectory has been set")

    # Stop current trajectory
    def stop_trajectory(self, msg=None):
        if not self.is_in_trajectory :
            self.get_logger().info("Not in trayectory, nothing to stop")
            return
        
        self.is_in_trajectory = False 
        self.current_setpoint_index = 0

    # Start current set trajectory
    def start_trajectory(self, msg=None):
        if self.current_trajectory is None:
            self.get_logger().info("No trajectory has been set. Aborting")
            return
        
        if self.is_in_trajectory:
            self.get_logger().info("Already in trayectory, stop first to start a new one")
            return
        
        self.is_in_trajectory = True
        self.current_setpoint_index = 0
        self.takeoff_publisher.publish(Empty())
        self.get_logger().info("Taking Off")

        thread = Thread(target=self.do_trajectory, args=(), daemon=True)
        thread.start()

    # Function for sending trajectory in a thread
    def do_trajectory(self):
        
        # Do trayectory
        for index, current_pose_st in enumerate(self.current_trajectory.poses):

            # Publish index
            msg = Int32()
            msg.data = index
            self.trajectory_progress_publisher.publish(msg)
            self.get_logger().info(f"Going to setpoint {index}/{len(self.current_trajectory.poses)-1}")

            # Get Previous Pose or move dron from current pos to first item in trajectory
            prev_pose_st = PoseStamped()
            prev_yaw = 0
            if index == 0:
                prev_pose_st.pose.position.x = self.drone_pose.pose.position.x
                prev_pose_st.pose.position.y = self.drone_pose.pose.position.y
                prev_pose_st.pose.position.z = self.drone_pose.pose.position.z
                _, _, prev_yaw = self.euler_from_quaternion(self.drone_pose.pose.orientation)
            else:
                prev_pose_st.pose.position.x = self.current_trajectory.poses[index-1].pose.position.x
                prev_pose_st.pose.position.y = self.current_trajectory.poses[index-1].pose.position.y
                prev_pose_st.pose.position.z = self.current_trajectory.poses[index-1].pose.position.z
                _, _, prev_yaw = self.euler_from_quaternion(self.current_trajectory.poses[index-1].pose.orientation)

            # Get Next Pose
            next_pose_st = PoseStamped()
            next_yaw = 0
            next_pose_st.pose.position.x = current_pose_st.pose.position.x
            next_pose_st.pose.position.y = current_pose_st.pose.position.y
            next_pose_st.pose.position.z = current_pose_st.pose.position.z
            _, _, next_yaw = self.euler_from_quaternion(current_pose_st.pose.orientation)

            # Handle last pose
            final = False
            if index == len(self.current_trajectory.poses)-1:
                final = True

            # Fill in gaps for smooth trayectory for translation
            x1, y1, z1 = prev_pose_st.pose.position.x, prev_pose_st.pose.position.y, prev_pose_st.pose.position.z
            x2, y2, z2 = next_pose_st.pose.position.x, next_pose_st.pose.position.y, next_pose_st.pose.position.z
            distance = math.sqrt((x2-x1)**2 + (y2-y1)**2 + (z2-z1)**2)
            time_to = distance / self.trajectory_velocity
            steps = math.floor(time_to / self.node_dt)
            xs = numpy.linspace(x1, x2, num=steps, endpoint=final)
            ys = numpy.linspace(y1, y2, num=steps, endpoint=final)
            zs = numpy.linspace(z1, z2, num=steps, endpoint=final)

            # Maintain yaw angles between -pi to pi
            if prev_yaw > math.pi: prev_yaw -= 2*math.pi
            if prev_yaw < -math.pi: prev_yaw += 2*math.pi
            if next_yaw > math.pi: next_yaw -= 2*math.pi
            if next_yaw < -math.pi: next_yaw += 2*math.pi

            # Handle cross over pi to -pi and viceversa
            if abs(next_yaw - prev_yaw) > math.pi:
                lim = 2*math.pi
                if prev_yaw < 0: lim = -lim
                next_yaw = lim + next_yaw
                yaws = numpy.linspace( prev_yaw, next_yaw, num=steps, endpoint=final )
                for i, element in enumerate(yaws):
                    if abs(element) > math.pi:
                        yaws[i] = element + lim
            else:
                yaws = numpy.linspace(prev_yaw, next_yaw, num=steps, endpoint=final)
            
            # Publish pose
            for i in range(steps):
                publish_pose_st = PoseStamped()
                publish_pose_st.header.frame_id = "ned_earth"
                publish_pose_st.header.stamp = self.get_clock().now().to_msg()

                publish_pose_st.pose.position.x = xs[i]
                publish_pose_st.pose.position.y = ys[i]
                publish_pose_st.pose.position.z = zs[i]

                quat = self.quaternion_from_euler(roll=0.0, pitch=0.0, yaw=yaws[i])
                publish_pose_st.pose.orientation.w = quat[0]
                publish_pose_st.pose.orientation.x = quat[1]
                publish_pose_st.pose.orientation.y = quat[2]
                publish_pose_st.pose.orientation.z = quat[3]

                if self.is_in_trajectory:
                    self.pos_publisher.publish(publish_pose_st)
                    time.sleep(self.node_dt)
                else:
                    return
        
        self.get_logger().info("End of trajectory")
        self.stop_trajectory()
        self.land_publisher.publish(Empty())
        self.land_publisher.publish(Empty())
        self.land_publisher.publish(Empty())

def main(args = None):
    rclpy.init(args=args)
    trajectory_manager = TrajectoryManager()

    rclpy.spin(trajectory_manager)
    
    trajectory_manager.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(e)