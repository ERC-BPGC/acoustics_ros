#! /usr/bin/env python3

import rospy
from enum import Enum
from pra_utils import core
from acoustics_ros.msg import *

# Temporary test parameters NEED TO UPDATE
path_to_rcf = "../rcf/test.rcf"
# tf_source_to_mic 

class State(Enum):
    IDLE = 0
    RECOMPUTE = 1
    PUBLISH = 2

class AcousticsNode:
    def __init__(self):
        self.source_pos = Point3D()
        self.mic_pos = Point3D()
        self.input_signal = SignalArray()
        self.state = State.IDLE

        self.pub = rospy.Publisher('acoustics_node', SignalArray, queue_size=10)

        self.sub_pos = rospy.Subscriber('robot_loc', Point3D, AcousticsNode.position_callback)
        self.sub_signal = rospy.Subscriber('input_signal', SignalArray, AcousticsNode.signal_callback)

    def position_callback(self, msg1):
        if msg1 != self.source_pos:
            # Put transform for source to mic here
            self.source_pos = msg1
            self.mic_pos = [msg1.x, msg1.y, msg1.z + 0.05]
            self.state = State.RECOMPUTE
        else:
            self.state = State.PUBLISH

    def signal_callback(self, msg1):
        if msg1 != self.input_signal:
            self.input_signal = msg1
            self.state = State.RECOMPUTE
        else:
            self.state = State.PUBLISH
    
    def ComputeWaveform(self):
        self.room = core.ComplexRoom.from_rcf(path_to_rcf)
        self.room.add_source(position=self.source_pos, signal=self.input_signal)
        self.room.add_microphone_array(loc=self.mic_pos)

        self.room.simulate()
        self.rir = list(self.room.mic_array.signals[0])

        pub.publish(self.rir)

    def timer_callback(self, event=None):
        # will check state and either publish or recompute and publish
        if self.state == State.IDLE:
            pass
        elif self.state == State.RECOMPUTE:
            self.ComputeWaveform()
        else:
            self.pub.publish(self.rir)

if __name__ == '__main__':
    try:
        rospy.init_node('acoustics_node', anonymous=True)
        node = AcousticsNode()

        while not rospy.is_shutdown():
            rir = Point3DArray()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass