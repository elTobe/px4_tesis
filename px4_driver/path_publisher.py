import rclpy
import math
import time
from nav_msgs.msg import Path
from std_msgs.msg import Empty
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

class PathPubliser(Node):
    def __init__(self):
        super().__init__("path_publisher")
        self.get_logger().info("About to publish helicoidal path. ")

        self.path_publisher = self.create_publisher(Path, "/trajectory_manager/set_path", 10)
        self.takeoff_publisher = self.create_publisher(Empty, "/px4_driver/takeoff", 10)
        self.starter_publisher = self.create_publisher(Empty, "/trajectory_manager/start", 10)
        self.drone_path_resetter = self.create_publisher(Empty, "/px4_driver/reset_drone_path", 10)
        
        self.theta = 0
        self.radio = 1
        self.delta_ang = 0.25
        self.max_alt = 2.0
        self.altura = 1.0
        self.delta_alt = 0.025

        msg = Path()
        msg.header.frame_id = "takeoff_position"
        #msg.poses.append(PoseStamped())
        #init_pose_st = PoseStamped()
        #init_pose_st.pose.position.z = -self.altura
        #msg.poses.append(init_pose_st)
        while(self.altura < self.max_alt):
            pose_st = PoseStamped()
            pose_st.pose.position.x = math.cos(self.theta) * self.radio
            pose_st.pose.position.y = math.sin(self.theta) * self.radio
            pose_st.pose.position.z = float(-self.altura)
            msg.poses.append(pose_st)

            self.altura += self.delta_alt
            self.theta += self.delta_ang

        self.path_publisher.publish(msg)
        time.sleep(0.1)

        self.drone_path_resetter.publish(Empty())
        time.sleep(0.1)

        self.get_logger().info("Taking Off")
        self.takeoff_publisher.publish(Empty())
        time.sleep(10)
        
        self.get_logger().info("Starting trajectory")
        self.starter_publisher.publish(Empty())

def main(args = None):
    rclpy.init(args=args)
    path_publisher = PathPubliser()

    path_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(e)