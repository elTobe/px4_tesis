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
        self.declare_parameter("frame", "takeoff_position")

        # Publishers
        self.path_publisher = self.create_publisher(Path, "/trajectory_manager/set_path", 10)
        self.takeoff_publisher = self.create_publisher(Empty, "/px4_driver/takeoff", 10)
        self.starter_publisher = self.create_publisher(Empty, "/trajectory_manager/start", 10)
        self.drone_path_resetter = self.create_publisher(Empty, "/px4_driver/reset_drone_path", 10)
        
        # Init Variables
        msg = Path()
        selected_frame = self.get_parameter("frame").get_parameter_value().string_value
        msg.header.frame_id = selected_frame
        self.get_logger().info(f"Selected frame : {selected_frame}")

        # Make selected path
        selected_path = self.get_parameter("path").get_parameter_value().string_value
        self.get_logger().info(f"Selected Path : {selected_path}")

        if selected_path == "cubo":

            # crete cube trayectory, not changing heading
            trajectory = [[0,0,-1], [0,1,-1], [1,1,-1], [1,0,-1], [0,0,-1], 
                          [0,0,-2], [0,1,-2], [1,1,-2], [1,0,-2], [0,0,-2]]
            
            for point in trajectory:
                pose_st = PoseStamped()
                pose_st.pose.position.x = float(point[0])
                pose_st.pose.position.y = float(point[1])
                pose_st.pose.position.z = float(point[2])
                msg.poses.append(pose_st)

        elif selected_path == "poly":
            
            # Create polygon trayectory facing inwards
            num_lados = 5
            grados, paso = numpy.linspace(0, 360, num_lados + 1, endpoint=True, retstep=True)

            for i, grad in enumerate(grados):
                pose_st = PoseStamped()
                pose_st.pose.position.x = math.cos(grad * math.pi/180)
                pose_st.pose.position.y = math.sin(grad * math.pi/180)
                pose_st.pose.position.z = -1.0

                q = self.quaternion_from_euler(yaw=(grad - 180)*math.pi/180)
                pose_st.pose.orientation.w = q[0]
                pose_st.pose.orientation.x = q[1]
                pose_st.pose.orientation.y = q[2]
                pose_st.pose.orientation.z = q[3]

                msg.poses.append(pose_st)
                msg.poses.append

        elif selected_path == "heli":

            # Create helicoidal path facing outwards
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
                q = self.quaternion_from_euler(yaw=self.theta)
                pose_st.pose.orientation.w = q[0]
                pose_st.pose.orientation.x = q[1]
                pose_st.pose.orientation.y = q[2]
                pose_st.pose.orientation.z = q[3]
                msg.poses.append(pose_st)

                self.altura += self.delta_alt
                self.theta += self.delta_ang

        # Make sinusoidal path
        elif selected_path == "sinu":
            paso = 10
            longitud = 2
            for i in range(0, 360, paso):
                pose_st = PoseStamped()
                pose_st.pose.position.x = longitud*i/360
                pose_st.pose.position.y = math.sin(i*math.pi/180) * 0.5
                pose_st.pose.position.z = 0.0
                q = self.quaternion_from_euler(yaw=0)
                pose_st.pose.orientation.w = -q[0]
                pose_st.pose.orientation.x = q[1]
                pose_st.pose.orientation.y = q[2]
                pose_st.pose.orientation.z = q[3]
                msg.poses.append(pose_st)
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
        #self.get_logger().info(f"Quat is : {q}")
        return q

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