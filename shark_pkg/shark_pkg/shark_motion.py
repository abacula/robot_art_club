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
import random


class SharkMotion(Node):
    def __init__(self):
        super().__init__('shark_node')

        robot_namespace = '/robot2'

        self.SOUND = False                          # do we want sounds
        self.JAWS_MODE = False                     # are we jaws


        self.LIGHTS = False                         # do we want lights
        self.BASE_LIGHTS = 'base'
        self.JAWS_LIGHTS = 'jaws'

        self.FORWARD_SPD = 0.5                             # m/s
        self.TURN_SPEED = 0.25
        self.CANDY_PAUSE = False

        self.CONF_THRESH = 0.0                             # min confidence
        self.TRIGGER_HEIGHT = 30                           # bbox_height that starts the slowdown
        self.PERSON_DETECTED = False
        self.person_x = 0.0
        self.person_height = 0.0
        
        self.PI = 3.14159265358979323846
        self.ANG = 0
        self.ANG_OFFSET = 0
        self.GOT_OFFSET = False         

        self.OBSTACLE_DETECTED = False              # stop movement if obstacle detected
        self.OBS_THRESH = 0.8                       # m, distance to obstacle that triggers stop

        self.RATE = 0.2                             # in seconds
        self. PAUSE_TIME = 5                        # in seconds
        self.PAUSE_ITERS = self.PAUSE_TIME / self.RATE
        self.iter = 0

        self.vel_pub = self.create_publisher(Twist, robot_namespace + '/cmd_vel_unstamped', 10)
        self.sound_pub = self.create_publisher(AudioNoteVector, robot_namespace + '/cmd_audio', 2)
        self.light_pub = self.create_publisher(String, '/light_state', 10)
        self.jaws_pub = self.create_publisher(String, robot_namespace + '/jaws_mode', 10)

        self.ack_sub = self.create_subscription(HallwayAck, robot_namespace + '/hallway_ack', self.detection_cb, 10)
        self.scan_sub = self.create_subscription(LaserScan, robot_namespace + '/scan', self.scan_cb, 10)
        self.odom_sub = self.create_subscription(Odometry, robot_namespace + '/odom', self.odom_cb, 10)

        self.time_timer = self.create_timer(self.RATE, self.time_cb)

    # ================================================================================
    # PERSON DETECTION
    # ================================================================================
    def detection_cb(self, msg):
        if (msg.person_detected
                and msg.confidence >= self.CONF_THRESH
                and msg.bbox_height > self.TRIGGER_HEIGHT):
            self.PERSON_DETECTED = True
            self.person_x = msg.position.x
            self.person_height = msg.bbox_height
            self.get_logger().info("Person detected")
            # every person has a 30% chance of being jawsed
            # jaws_test = random.randint(0, 2)
            # if (jaws_test == 2):
            self.JAWS_MODE = True
        else: 
            self.PERSON_DETECTED = False
        

    def time_cb(self):
        twist = Twist()
        turn_left = False
        turn_right = False

        if self.CANDY_PAUSE:
            if self.iter < self.PAUSE_ITERS:
                self.iter += 1
            else:
                self.iter = 0
                self.CANDY_PAUSE = False


        if self.JAWS_MODE:
            is_straight = self.person_x - 125
            self.get_logger().info("Person detected ahead at " + str(self.person_x))

            if abs(is_straight) < 15:
                d_ang = self.ANG
                turn_left = False
                turn_right = False
    
            elif is_straight < 0:
                #turn left
                d_ang = self.ANG
                turn_left = True 
                turn_right = False
            else:
                #turn right
                d_ang = self.ANG - 0.5
                turn_left = False
                turn_right = True
                
        else:
            #go forward at set speed and sometimes rotate
            random_walk = random.randint(0, 5)
            if (random_walk == 0): 
                random_turn = self.PI * random.randint(-40,40) / 180
                self.get_logger().info("Doing random turn.")

                d_ang = self.ANG + random_turn
            else: 
                d_ang = self.ANG

        
        # orient to correct angle 
        if ((self.ANG > d_ang) or turn_right):
            twist.angular.z = -self.TURN_SPEED
            self.get_logger().info("Turning right to correct orientation.")
        elif ((self.ANG < -d_ang) or turn_left):
            twist.angular.z = self.TURN_SPEED
            self.get_logger().info("Turning left to correct orientation.")
        else:
            twist.angular.z = 0.0
           
        
        # change lights
        if self.LIGHTS:
            self.change_lights()
            
        if self.OBSTACLE_DETECTED:
            twist.linear.x = 0.0
            self.get_logger().warn("Obstacle detected -- stopping movement.")
            if self.PERSON_DETECTED:
                self.CANDY_PAUSE = True
            else: 
                # Turn to move away from obstacle
                twist.angular.z = self.TURN_SPEED
        elif self.GOT_OFFSET is False:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.get_logger().warn("Waiting for odometry to be set.")
        elif self.CANDY_PAUSE:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.get_logger
        else:
            twist.linear.x = self.FORWARD_SPD
            self.get_logger().info("Moving Forward.")

        self.vel_pub.publish(twist)

    # ================================================================================
    # OBSTALCE DETECTION
    # ================================================================================
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
                self.JAWS_MODE = False
                self.get_logger().warn("Obstacle detected -- stopping movement.")
                break
    
    # ================================================================================
    # ODOMETRY
    # ================================================================================
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
        if self.JAWS_MODE:
            light_msg.data = self.JAWS_LIGHTS
        else:
            light_msg.data = self.BASE_LIGHTS
        
        self.light_pub.publish(light_msg)




def main(args=None):
    rclpy.init(args=args)
    node = SharkMotion()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()