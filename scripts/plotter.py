import rospy
import matplotlib.pyplot as plt
from acoustics_ros.msg import SignalArray
import time

# consider this https://answers.ros.org/question/259086/plotting-data-with-mayavi-via-the-subscriber-node/ 

new_rir_received = False
last_rir = None
def signal_callback(msg):
	global last_rir, new_rir_received
	rir = msg.signals[0].array

	if rir != last_rir:
		last_rir = rir
		new_rir_received = True
		
sub = rospy.Subscriber('/acoustic_signal', SignalArray, signal_callback)

try: 
	rospy.init_node('plotter')

	r = rospy.Rate(5)

	plt.ion()	# turn on interactive plot
	plt.pause(0.1)

	rospy.loginfo('Ready to plot.')
	while not rospy.is_shutdown():
		if new_rir_received is True:
			new_rir_received = False

			rospy.loginfo('Plotting new data...')
			plt.cla()
			plt.grid(True)
			plt.plot(last_rir)
			plt.pause(0.1)

		r.sleep()


except rospy.ROSInterruptException:
	pass