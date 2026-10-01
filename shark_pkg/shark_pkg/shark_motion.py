import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from yolo_msgs.msg import HallwayAck
from irobot_create_msgs.msg import AudioNote, AudioNoteVector
from builtin_interfaces.msg import Duration
from std_msgs.msg import String
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion


class SharkMotion(Node):
    def __init__(self):
        super().__init__('slow_node_MLS')

        robot_namespace = '/robot2'

        self.SOUND = True                          # do we want sounds
        self.JAWS_MODE = False                     # are we jaws
        self.LIGHTS = True                         # do we want lights

        self.FORWARD_LIGHTS = "instant 85 1"                 # light state when moving at normal speed
        self.SLOW_LIGHTS = "fade 5 42 5"                     # light state when moving at slow speed

        self.FORWARD_SPD = 0.5                             # m/s
        self.SLOW_SPEED = self.FORWARD_SPD / 2             # m/s
        self.CONF_THRESH = 0.0                             # min confidence
        self.TRIGGER_HEIGHT = 30                           # bbox_height that starts the slowdown
        
        self.ANG_THRESH = 0.02                       # rad, angle that triggers stop
        self.PI = 3.141592653589793
        self.ANG = 0
        self.ANG_OFFSET = 0
        self.GOT_OFFSET = False

        self.CURR_SPEED = 0.0           

        self.OBSTACLE_DETECTED = False              # stop movement if obstacle detected
        self.OBS_THRESH = 0.75                       # m, distance to obstacle that triggers stop

        self.publisher = self.create_publisher(Twist, robot_namespace + '/cmd_vel_unstamped', 10)
        self.ack_sub = self.create_subscription(HallwayAck, robot_namespace + '/hallway_ack', self.hallway_cb, 10)
        self.sound_pub = self.create_publisher(AudioNoteVector, robot_namespace + '/cmd_audio', 2)
        self.light_pub = self.create_publisher(String, '/light_state', 10)
        self.scan_sub = self.create_subscription(LaserScan, robot_namespace + '/scan', self.scan_cb, 10)
        self.odom_sub = self.create_subscription(Odometry, robot_namespace + '/odom', self.odom_cb, 10)

    def hallway_cb(self, msg):
        twist = Twist()
        if (msg.person_detected
                and msg.confidence >= self.CONF_THRESH
                and msg.bbox_height > self.TRIGGER_HEIGHT):
            self.get_logger().info("Person detected")

        
                   
        else:
            self.get_logger().info("No person detected -- keep moving.")
            twist.linear.x = self.FORWARD_SPD # keep moving at normal speed

        if self.ANG > self.ANG_THRESH:
            twist.angular.z = -0.15
            self.get_logger().info("Turning right to correct orientation.")
        elif self.ANG < -self.ANG_THRESH:
            twist.angular.z = 0.15
            self.get_logger().info("Turning left to correct orientation.")
        else:
            self.get_logger().info(str(round(self.ANG,3)))

        self.CURR_SPEED = twist.linear.x
        
        # change lights
        if self.LIGHTS:
            self.change_lights()
            
        if self.OBSTACLE_DETECTED:
            twist.linear.x = 0.0
            self.get_logger().warn("Obstacle detected -- stopping movement.")
        elif self.GOT_OFFSET is False:
            twist.linear.x = 0.0
            self.get_logger().warn("Waiting for odometry to be set.")

        self.publisher.publish(twist)


    def scan_cb(self, msg):
        self.OBSTACLE_DETECTED = False
        front_ranges = msg.ranges[200:340]
        min = msg.range_min
        max = msg.range_max
        for distance in front_ranges:
            if distance <= min or distance >= max:
                continue

            if distance < self.OBS_THRESH:
                self.OBSTACLE_DETECTED = True
                self.get_logger().warn("Obstacle detected -- stopping movement.")
                break
        
                

    def odom_cb(self, msg):
        quaternion = msg.pose.pose.orientation
         # Angle converted from quaternion to euler
        (_,_,ang) = euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])

        if self.GOT_OFFSET is False:
            self.ANG_OFFSET = -ang
            self.GOT_OFFSET = True

        ang += self.ANG_OFFSET
        
        if ang < -self.PI:
            self.ANG = ang + (2*self.PI)
        else:
            self.ANG = ang
    
    # ================================================================================
    # LIGHTS
    # ================================================================================
    def change_lights(self):
        light_msg = String()
        if self.CURR_SPEED < self.FORWARD_SPD:
            light_msg.data = self.SLOW_LIGHTS
        else:
            light_msg.data = self.FORWARD_LIGHTS
        self.light_pub.publish(light_msg)

    # ================================================================================
    # SOUNDS
    # ================================================================================
    def change_sounds(self):
        audio_msg = AudioNoteVector()
        Melody = [1174, 1318, 1568]
        Durations = [.2, .2, .4]
        for x in range(len(Melody)):
            note = AudioNote()           
            time_play = Duration()

            time_play.nanosec = int(Durations[x] * 1000000000) # val * 1 second
            note.max_runtime = time_play
            note.frequency = Melody[x]

            audio_msg.append = True
            audio_msg.notes.append(note)

        self.sound_pub.publish(audio_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SlowdownMovementMLS()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()