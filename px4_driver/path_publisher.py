import rclpy
import math
import time
import numpy
from nav_msgs.msg import Path
from std_msgs.msg import Empty
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

class PathPubliser(Node):
    def __init__(self):
        super().__init__("path_publisher")
        self.get_logger().info("About to publish path. ")

        # Parameters
        self.declare_parameter("path", "heli")

        # Publishers
        self.path_publisher = self.create_publisher(Path, "/trajectory_manager/set_path", 10)
        self.takeoff_publisher = self.create_publisher(Empty, "/px4_driver/takeoff", 10)
        self.starter_publisher = self.create_publisher(Empty, "/trajectory_manager/start", 10)
        self.drone_path_resetter = self.create_publisher(Empty, "/px4_driver/reset_drone_path", 10)
        
        # Init Variables
        msg = Path()
        msg.header.frame_id = "takeoff_position"

        # Make selected path
        selected_path = self.get_parameter("path").get_parameter_value().string_value
        self.get_logger().info(f"Selected Path : {selected_path}")

        if selected_path == "cubo":

            trajectory = [[0,0,-1], [0,1,-1], [1,1,-1], [1,0,-1], [0,0,-1], 
                          [0,0,-2], [0,1,-2], [1,1,-2], [1,0,-2], [0,0,-2]]
            
            for point in trajectory:
                pose_st = PoseStamped()
                pose_st.pose.position.x = float(point[0])
                pose_st.pose.position.y = float(point[1])
                pose_st.pose.position.z = float(point[2])
                msg.poses.append(pose_st)

        elif selected_path == "poly":
            
            num_lados = 5
            grados = numpy.linspace(0, 360, num_lados + 1, endpoint=True)

            for grad in grados:
                pose_st = PoseStamped()
                pose_st.pose.position.x = math.cos(grad * math.pi/180)
                pose_st.pose.position.y = math.sin(grad * math.pi/180)
                pose_st.pose.position.z = -1.0
                msg.poses.append(pose_st)

        elif selected_path == "heli":

            # Create helicoidal path
            self.theta = 0
            self.radio = 1
            self.delta_ang = 0.25
            self.max_alt = 2.0
            self.altura = 1.0
            self.delta_alt = 0.025
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

        else:
            self.get_logger().info("No valid path, exiting")
            return

        # Publish Path
        self.path_publisher.publish(msg)
        time.sleep(0.1)

        # Reset drone position history for logging purposes
        self.drone_path_resetter.publish(Empty())
        time.sleep(0.1)

        # Do takeoff
        self.get_logger().info("Taking Off")
        self.takeoff_publisher.publish(Empty())
        time.sleep(10)

        # Start trajectory
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