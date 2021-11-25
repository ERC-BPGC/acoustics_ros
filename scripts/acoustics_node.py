#! /usr/bin/env python3

import rospy
from pra_utils import core
from acoustics_ros.msg import *

# Temporary test parameters NEED TO UPDATE
path_to_rcf = "../rcf/test.rcf"

class AcousticsNode:
    def __init__(self):
        self.source_pos = Point3D()
        self.mic_pos = Point3D()
        self.input_signal = Point3DArray()

    def position_callback(self, msg1):
        self.source_pos = msg1
        self.mic_pos = [msg1.x, msg1.y, msg1.z + 0.05]

        self.ComputeWaveform()

    def signal_callback(self, msg1):
        self.input_signal = msg1
        
        self.ComputeWaveform()

    def ComputeWaveform(self):
        self.room = core.ComplexRoom.from_rcf(path_to_rcf)
        self.room.add_source(position=self.source_pos, signal=self.input_signal)
        self.room.add_microphone_array(loc=self.mic_pos)

        self.room.simulate()
        self.rir = list(self.room.mic_array.signals[0])

        pub.publish(self.rir)

if __name__ == '__main__':
    try:
        rospy.init_node('acoustics_node', anonymous=True)
        pub = rospy.Publisher('acoustics_node', Point3DArray, queue_size=10)
        rate = rospy.Rate(1)  #1hz

        while not rospy.is_shutdown():
            rir = Point3DArray()
            rospy.Subscriber('robot_loc', Point3D, AcousticsNode.position_callback)
            rospy.Subscriber('input_signal', Point3DArray, AcousticsNode.signal_callback)
            rate.sleep()

    except rospy.ROSInterruptException:
        pass