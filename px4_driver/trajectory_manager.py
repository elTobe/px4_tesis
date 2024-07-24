import rclpy
import time
import math
import numpy
from rclpy.node import Node
from threading import Thread
from nav_msgs.msg import Path
from std_msgs.msg import Float32, Empty, Int32
from geometry_msgs.msg import Pose, PoseStamped

class TrajectoryManager(Node):
    def __init__(self):
        super().__init__("trajectory_manager")
        self.get_logger().info("Starting trajectory manager node - ...")

        # Initialize Variables
        self.trajectory_velocity = 0.3
        self.takeoff_time = 8
        self.drone_pose = PoseStamped()
        self.current_setpoint_index = 0
        self.current_trajectory = None
        self.is_in_trajectory = False
        self.take_off_position = PoseStamped()
        self.node_rate = 10
        self.node_dt = 1/self.node_rate

        # Create Publishers
        self.trajectory_progress_publisher = self.create_publisher(Int32, "/trajectory_manager/current_index", 10)
        self.path_publisher = self.create_publisher(Path, "/trajectory_manager/current_path", 10)
        self.takeoff_publisher = self.create_publisher(Empty, "/px4_driver/takeoff", 10)
        self.pos_publisher = self.create_publisher(Pose, "/px4_driver/cmd_pos", 10)

        # Create Subscribers
        self.takeoff_position_subscriber = self.create_subscription(PoseStamped, "/px4_driver/takeoff_pose", self.takeoff_position_update,10)
        self.drone_pose_subscriber = self.create_subscription(PoseStamped, "/px4_driver/drone_pose", self.drone_pose_update,10)
        self.set_trajectory_subscriber = self.create_subscription(Path, "/trajectory_manager/set_path", self.set_trajectory, 10)
        self.set_velocity_subscriber = self.create_subscription(Float32, "/trajectory_manager/set_velocity", self.set_velocity, 10)
        self.start_trajectory_subscriber = self.create_subscription(Empty, "/trajectory_manager/start", self.start_trajectory, 10)
        self.stop_trajectory_subscriber = self.create_subscription(Empty, "/trajectory_manager/stop", self.stop_trajectory, 10)

    # Update current drone pose
    def drone_pose_update(self, msg):
        self.drone_pose = msg

    def takeoff_position_update(self, msg):
        self.take_off_position = msg

    # Set velocity for trayectory following
    def set_velocity(self, msg):
        self.velocity = msg.data
        self.get_logger().info(f"Velocity has been set to {self.velocity}")

    # Set ned trajectory
    def set_trajectory(self, msg):
        self.current_trajectory = msg
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
        for index, next_pose_st in enumerate(self.current_trajectory.poses):

            # Publish index
            msg = Int32()
            msg.data = index
            self.trajectory_progress_publisher.publish(msg)

            # Move dron from current pos to first item in trajectory
            prev_pose_st = PoseStamped()
            if index == 0:
                
                prev_pose_st.pose.position.x = self.drone_pose.pose.position.x - self.take_off_position.pose.position.x
                prev_pose_st.pose.position.y = self.drone_pose.pose.position.y - self.take_off_position.pose.position.y
                prev_pose_st.pose.position.z = self.drone_pose.pose.position.z - self.take_off_position.pose.position.z
            else:
                prev_pose_st = self.current_trajectory.poses[index-1]

            # Handle last pose
            final = False
            if index == len(self.current_trajectory.poses)-1:
                final = True

            # Fill in gaps for smooth trayectory
            x1, y1, z1 = prev_pose_st.pose.position.x, prev_pose_st.pose.position.y, prev_pose_st.pose.position.z
            x2, y2, z2 = next_pose_st.pose.position.x, next_pose_st.pose.position.y, next_pose_st.pose.position.z
            distance = math.sqrt((x2-x1)**2 + (y2-y1)**2 + (z2-z1)**2)
            time_to = distance / self.trajectory_velocity
            steps = math.floor(time_to / self.node_dt)
            xs = numpy.linspace(x1, x2, num=steps, endpoint=final)
            ys = numpy.linspace(y1, y2, num=steps, endpoint=final)
            zs = numpy.linspace(z1, z2, num=steps, endpoint=final)

            # Publish pose
            for i in range(steps):
                publish_pose = Pose()
                publish_pose.orientation = next_pose_st.pose.orientation
                publish_pose.position.x = xs[i]
                publish_pose.position.y = ys[i]
                publish_pose.position.z = zs[i]

                if self.is_in_trajectory:
                    self.pos_publisher.publish(publish_pose)
                    self.path_publisher.publish(self.current_trajectory)
                    time.sleep(self.node_dt)
                else:
                    return
        self.stop_trajectory()

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