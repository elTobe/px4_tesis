import rclpy
import math
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

class PathPubliser(Node):
    def __init__(self):
        super().__init__("path_publisher")
        self.get_logger().info("About to publish helicoidal path. ")

        self.path_publisher = self.create_publisher(Path, "/trajectory_manager/set_path", 10)
        
        self.theta = 0
        self.radio = 1
        self.delta_ang = 0.25
        self.max_alt = 2.0
        self.altura = 1.0
        self.delta_alt = 0.025

        msg = Path()
        msg.header.frame_id = "local_home"
        while(self.altura < self.max_alt):
            pose = PoseStamped()
            pose.pose.position.x = math.cos(self.theta) * self.radio
            pose.pose.position.y = math.sin(self.theta) * self.radio
            pose.pose.position.z = float(-self.altura)
            msg.poses.append(pose)

            self.altura += self.delta_alt
            self.theta += self.delta_ang

        self.path_publisher.publish(msg)

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