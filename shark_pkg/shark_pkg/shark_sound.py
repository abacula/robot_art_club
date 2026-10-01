import rclpy
from rclpy.node import Node
from irobot_create_msgs.msg import AudioNote, AudioNoteVector
from builtin_interfaces.msg import Duration
from std_msgs.msg import String
from rclpy.qos import qos_profile_sensor_data



class SharkSound(Node):
    def __init__(self):
        super().__init__('shark_sound_node')

        robot_namespace = '/robot2'

        # replace this sub with a msg for jaws sound
        self.jaws_sub = self.create_subscription(String, robot_namespace + '/jaws_mode', self.jaws_cb, 10)
        self.sound_pub = self.create_publisher(AudioNoteVector, robot_namespace + '/cmd_audio', 2)

    def jaws_cb(self,msg):
        pass
        # parse jaws info here

    # ================================================================================
    # SOUNDS
    # ================================================================================
    def change_sounds(self):
        audio_msg = AudioNoteVector()

        E = 164
        F = 174
        P = 0

        Melody = [E, F, P]
        # mke these relative to person distance
        Durations = [.5, .1, 2] # in seconds
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