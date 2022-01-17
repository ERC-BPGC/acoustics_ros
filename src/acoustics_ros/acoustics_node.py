#! /usr/bin/env python3

from enum import Enum
from lib2to3.pytree import Base
from os.path import isfile

import rospy
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped, TransformStamped, Point32
from pra_utils.core import ComplexRoom

from acoustics_ros.msg import SignalArray


class SimState(Enum):
	IDLE = 0
	RECOMPUTE = 1
	READY = 2

class AcousticsNode:
	def __init__(self):
		
		self.state = SimState.IDLE

		try:
			self.path_to_rcf = rospy.get_param('path_to_rcf')
			if not isfile(self.path_to_rcf):
				raise FileNotFoundError(f'RCF file not found at {self.path_to_rcf}.')

		except BaseException as err:
			rospy.logerr(err)
			rospy.signal_shutdown('Error obtaining necessary values from parameter server!')

		# self.input_signal = SignalArray()
		self.rir = None

		self.signal_pub = rospy.Publisher('acoustic_signal', SignalArray, queue_size=10)

		self.mic_pos_sub = rospy.Subscriber('robot_pose', PoseStamped, AcousticsNode.mic_callback)
		self.source_pos_sub = rospy.Subscriber('source_pos', Point32, AcousticsNode.source_callback)

		self.last_mic_pos = Point32()
		self.last_source_pos = Point32()
		# self.sub_signal = rospy.Subscriber('input_signal', SignalArray, AcousticsNode.signal_callback)

	def mic_callback(self, msg):
		if msg != self.last_mic_pos:
			self.last_mic_pos = msg
		
			self.state = SimState.RECOMPUTE
		else:
			self.state = SimState.READY
	
	def source_callback(self, msg):
		if msg != self.last_source_pos:
			self.last_source_pos = msg
		
			self.state = SimState.RECOMPUTE
		else:
			self.state = SimState.READY

	def timer_callback(self, event=None):
		"""For periodically publishing simulated waveform. """
	
		if self.state == SimState.RECOMPUTE:
			self.compute_waveform()
			self.state = SimState.READY
		
		if self.state == SimState.READY:
			self.signal_pub.publish(self.rir)

	# def signal_callback(self, msg1):
	# 	if msg1 != self.input_signal:
	# 		self.input_signal = msg1
	# 		self.state = State.RECOMPUTE
	# 	else:
	# 		self.state = State.PUBLISH
	
	def compute_waveform(self):
		self.room = ComplexRoom.from_rcf(self.path_to_rcf)
		self.room.add_source(position=self.last_source_pos)
		self.room.add_microphone(self.last_mic_pos)

		self.room.compute_rir()
		self.rir = list(self.room.rir[0][0])

