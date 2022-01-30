#! /usr/bin/env python3

from enum import Enum
from os.path import isfile

import rospy
from geometry_msgs.msg import Point32
from pra_utils.core import ComplexRoom

from acoustics_ros.msg import SignalArray, FloatArray

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
			else:
				rospy.loginfo(f'Using RCF {self.path_to_rcf}.')

		except BaseException as err:
			rospy.logerr(err)
			rospy.signal_shutdown('Error obtaining necessary values from parameter server!')

		# self.input_signal = SignalArray()
		self.rirs = SignalArray()

		self.signal_pub = rospy.Publisher('acoustic_signal', SignalArray, queue_size=10)

		self.mic_pos_sub = rospy.Subscriber('mic_pos', Point32, self.mic_callback)
		self.source_pos_sub = rospy.Subscriber('source_pos', Point32, self.source_callback)

		self.pub_timer = rospy.Timer(rospy.Duration(1), self.timer_callback)

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
			self.signal_pub.publish(self.rirs)

	# def signal_callback(self, msg1):
	# 	if msg1 != self.input_signal:
	# 		self.input_signal = msg1
	# 		self.state = State.RECOMPUTE
	# 	else:
	# 		self.state = State.PUBLISH
	
	def compute_waveform(self):
		self.room = ComplexRoom.from_rcf(self.path_to_rcf)
		srcpos = point_to_list(self.last_source_pos)
		micpos = point_to_list(self.last_mic_pos)
		rospy.loginfo(f'src: {srcpos}, mic: {micpos}')

		self.room.add_source(srcpos)
		self.room.add_microphone(micpos)

		self.room.compute_rir()
		rirarray = FloatArray()
		rirarray.array = list(self.room.rir[0][0])
		self.rirs.signals = [rirarray]

		rospy.loginfo(f'self.rirs.signals: len = {len(self.rirs.signals)}, type = {type(self.rirs.signals)}')

def point_to_list(p: Point32):
	"""Converts geometry_msgs.msg.Point to list[3]"""
	return [p.x, p.y, p.z]

