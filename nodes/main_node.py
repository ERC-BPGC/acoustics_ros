from acoustics_ros.acoustics_node import AcousticsNode

import rospy
import os

if __name__ == '__main__':
	try:
		rospy.init_node('acoustics_node', anonymous=False)
		node = AcousticsNode()

		rospy.spin()

	except rospy.ROSInterruptException:
		pass