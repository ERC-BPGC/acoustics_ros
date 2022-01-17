#! /usr/bin/env python3

from enum import Enum
from os.path import isfile

import rospy
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped, TransformStamped
from pra_utils.core import ComplexRoom

from acoustics_ros.msg import SignalArray


class SimState(Enum):
	IDLE = 0
	RECOMPUTE = 1
	READY = 2

class AcousticsNode:
	def __init__(self):
		
		self.state = SimState.IDLE

		self.tf_robot_to_source = rospy.get_param('tf_robot_to_source')
		self.tf_robot_to_mic = rospy.get_param('tf_robot_to_mic')

		self.path_to_rcf = rospy.get_param('path_to_rcf')
		if not isfile(self.path_to_rcf):
			rospy.logerr('Rcf not found.')
			rospy.signal_shutdown()

		self.last_robot_pose = None
		self.source_pos = None
		self.mic_pos = None

		# self.input_signal = SignalArray()
		self.rir = None

		self.pub = rospy.Publisher('acoustic_signal', SignalArray, queue_size=10)

		self.sub_pos = rospy.Subscriber('robot_pose', PoseStamped, AcousticsNode.pose_callback)
		# self.sub_signal = rospy.Subscriber('input_signal', SignalArray, AcousticsNode.signal_callback)

	def pose_callback(self, msg1):
		"""Called when new pose data received. """
		if msg1.pose != self.last_robot_pose:
			rospy.loginfo("New pose!")
			self.last_robot_pose = msg1.pose

			self.source_pos = tf2_geometry_msgs.do_transform_pose(self.last_robot_pose, self.tf_robot_to_source)
			self.mic_pos = tf2_geometry_msgs.do_transform_pose(self.last_robot_pose, self.tf_robot_to_mic)

			self.state = SimState.RECOMPUTE
		else:
			self.state = SimState.READY

	def timer_callback(self, event=None):
		"""For periodically publishing simulated waveform. """
	
		if self.state == SimState.RECOMPUTE:
			self.compute_waveform()
			self.state = SimState.READY
		
		if self.state == SimState.READY:
			self.pub.publish(self.rir)

	# def signal_callback(self, msg1):
	# 	if msg1 != self.input_signal:
	# 		self.input_signal = msg1
	# 		self.state = State.RECOMPUTE
	# 	else:
	# 		self.state = State.PUBLISH
	
	def compute_waveform(self):
		self.room = ComplexRoom.from_rcf(self.path_to_rcf)
		self.room.add_source(position=self.source_pos, signal=self.input_signal)
		self.room.add_microphone_array(loc=self.mic_pos)

		self.room.compute_rir()
		self.rir = list(self.room.rir[0][0])

