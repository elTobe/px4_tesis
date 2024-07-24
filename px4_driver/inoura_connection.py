import rclpy
import time
from rclpy.node import Node
from pybleno import *
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Empty, Int32, Float32


# General Class for an output characteristic with values of type int 
class OutputChar(Characteristic):
    def __init__(self, name, uuid):
        options = {
            'uuid': uuid,
            'properties': ['read', 'notify'],
            'value': None
        }
        self.name = name
        self._value = str(0).encode()
        self._updateValueCallback = None
        super().__init__(options)

    def onReadRequest(self, offset, callback):
        print(f'{self.name} - onReadRequest')
        callback(result=super().RESULT_SUCCESS, data=self._value)

    def onSubscribe(self, maxValueSize, updateValueCallback):
        print(f'{self.name} - onSubscribe')
        self._updateValueCallback = updateValueCallback

    def onUnsubscribe(self):
        print(f'{self.name} - onUnsubscribe')
        self._updateValueCallback = None
        
    def updateValue(self, value):
        print(f"{self.name} - Updating Value to : {value}")
        bytes = []
        init=False
        for i in reversed(range(8)):
            if((value & (0xFF<<i*8)) or init):
                bytes.append((value>>i*8) & 0xFF)
                init=True
        self._value = bytes
        if self._updateValueCallback:
            self._updateValueCallback(self._value)


# General Class for an input characteristic
class InputChar(Characteristic):
    def __init__(self, name, uuid, on_input_call=None):
        options = {
            'uuid': uuid,
            'properties': ['write', 'notify'],
            'value': None
        }
        self.name = name
        self.on_input_call = on_input_call
        super().__init__(options)

    def onWriteRequest(self, data, offset, withoutResponse, callback):
        if self.on_input_call is not None:
            self.on_input_call(data)
        callback(super().RESULT_SUCCESS)
        

# Node for receiving Inoura app trajectories
class InouraConnection(Node):

    def __init__(self):
        super().__init__("inoura_connection")
        self.get_logger().info("Starting Inoura Connection Node ... ")
        self.max_speed = 30
        
        # Create Publishers
        self.waypoint_pub = self.create_publisher(Path, "/trajectory_manager/set_path", 10)
        self.start_pub = self.create_publisher(Empty, "/trajectory_manager/start", 10)
        self.speed_pub = self.create_publisher(Float32, "/trajectory_manager/set_velocity", 10)
        self.stop_pub = self.create_publisher(Empty, "/trajectory_manager/stop", 10)
        
        # Create Subscribers
        self.index_sub = self.create_subscription(Int32, "/trajectory_manager/current_index", self.notify_progress, 10)

        # Create Characteristics map
        self.chars = {
            "stop" :            InputChar("Stop","27f169cf-0000-0001-0001-9f77842e61db", self.onStopRequest),
            "right_motor" :     InputChar("RightMotor", "27f169cf-0000-0001-0002-9f77842e61db"),
            "waypoints" :       InputChar("Waypoints","27f169cf-0000-0001-0003-9f77842e61db", self.onWaypointsRequest),
            "speed" :           InputChar("Speed","27f169cf-0000-0001-0004-9f77842e61db", self.onSpeedRequest),
            "rotate_angle" :    InputChar("RotateAngle","27f169cf-0000-0002-0001-9f77842e61db"),
            "side_angle" :      InputChar("SideAngle","27f169cf-0000-0002-0002-9f77842e61db"),
            "forward_angle" :   InputChar("ForwardAngle","27f169cf-0000-0002-0003-9f77842e61db"),
            "shoulder" :        InputChar("Shoulder","27f169cf-0000-0003-0001-9f77842e61db"),
            "arm" :             InputChar("Arm","27f169cf-0000-0003-0002-9f77842e61db"),
            "elbow" :           InputChar("Elbow","27f169cf-0000-0003-0003-9f77842e61db"),
            "head_angle" :      InputChar("HeadAngle","27f169cf-0000-0004-0001-9f77842e61db"),
            "neck_angle" :      InputChar("NeckAngle", "27f169cf-0000-0004-0002-9f77842e61db"),
            "gesture" :         InputChar("Gesture", "27f169cf-0000-0004-0003-9f77842e61db"),
            "leds_color" :      InputChar("LEDsColor", "27f169cf-0000-0005-0001-9f77842e61db"),
            "mute" :            InputChar("Mute", "27f169cf-0000-0005-0002-9f77842e61db"),
            "battery" :         OutputChar("Battery", "27f169cf-0000-0006-0001-9f77842e61db"),
            "index" :           OutputChar("WaypointIndex", "27f169cf-0000-0006-0002-9f77842e61db"),
            "sensor" :          OutputChar("SensorDistance", "27f169cf-0000-0006-0003-9f77842e61db"),
            "status" :          OutputChar("Status", "27f169cf-0000-0006-0004-9f77842e61db")
        }

        # Create Bleno
        self.bleno = Bleno()
        self.bleno.on('stateChange', self.onBlenoStateChange)
        self.bleno.on('advertisingStart', self.onBlenoAdvertisingStart)
        self.bleno.start()
    
    def onBlenoStateChange(self, state):
        self.get_logger().info('on -> stateChange: ' + state)
        if (state == 'poweredOn'):
            self.bleno.startAdvertising(name='Inoura', service_uuids=['27f169cf-0000-0000-0000-9f77842e61db'])
        else:
            self.bleno.stopAdvertising()

    def onBlenoAdvertisingStart(self, error):
        self.get_logger().info('on -> advertisingStart: ' + ('error ' + str(error) if error else 'success'))
        if not error:
            self.bleno.setServices([
                BlenoPrimaryService({
                    'uuid': '27f169cf-0000-0000-0000-9f77842e61db',
                    'characteristics': self.chars.values()
                })
            ])
        
    def onStopRequest(self, data):
        self.get_logger().info("Stop requested from app")
        self.stop_pub.publish(Empty())
    
    def onWaypointsRequest(self, data):
        self.get_logger().info(f"Trajectory from app received with {len(data)} bytes lenght")
        path = Path()
        path.header.frame_id = "local_home"
        for pos in range(4, len(data), 4): #ignore first item
            if pos+3 < len(data) and pos > 4: 
                pose_stamped = PoseStamped()
                pose = Pose()

                z = (data[pos] << 8) | data[pos+1]
                z = (z - 32768) % 65536 - 32768 + 250
                z = float(z / -100)

                x = (data[pos+2] << 8) | data[pos+3]
                x = (x - 32768) % 65536 - 32768
                x = float(x / 100)

                pose.position.y = -x
                pose.position.z = z
                pose_stamped.pose = pose
                path.poses.append(pose_stamped)

            else:
                self.get_logger().info("Ignoring last coordenate as it is not 4 bytes")

        self.get_logger().info(f"Waypoints are : { [[pose_stamped.pose.position.x, pose_stamped.pose.position.z] for pose_stamped in path.poses] } ")
        self.waypoint_pub.publish(path)

    
    def onSpeedRequest(self, data):
        speed = int(data[0]) * self.max_speed/255
        self.get_logger().info(f"Speed from app is { speed } cm/s")
        msg = Float32()
        msg.data = speed
        self.speed_pub.publish(msg)

        self.get_logger().info("Starting trajectory")
        self.start_pub.publish(Empty())

    def notify_progress(self, msg):
        self.chars["index"].updateValue(msg.data)

# Definition of main function to create and run the node
def main(args=None):
    rclpy.init(args=args)
    inoura_connect = InouraConnection()

    rclpy.spin(inoura_connect)

    inoura_connect.destroy_node()
    rclpy.shutdown()

# Call main function
if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)

