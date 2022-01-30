import rospy
from geometry_msgs.msg import Twist, Point32, Pose2D

## Very rough code, needs healthy dose of refactoring

robot_pose = Pose2D()
robot_pose.x = 5.0
robot_pose.y = 1.

pub_mic = rospy.Publisher('/mic_pos', Point32, queue_size=5)
pub_source = rospy.Publisher('/source_pos', Point32, queue_size=5)
pub_robot = rospy.Publisher('/robot_pose', Pose2D, queue_size=5)

def sub_callback(msg):
	robot_pose.x += msg.linear.x
	robot_pose.theta += msg.angular.z

	source_pos = Point32()
	source_pos.x = robot_pose.x
	source_pos.y = robot_pose.y
	source_pos.z = 1.

	mic_pos = Point32()
	mic_pos.x = source_pos.x
	mic_pos.y = source_pos.y
	mic_pos.z = source_pos.z + 0.1

	pub_source.publish(source_pos)
	pub_mic.publish(mic_pos)
	pub_robot.publish(robot_pose)

sub = rospy.Subscriber('/cmd', Twist, sub_callback)

def main():
	try:
		rospy.init_node('test_node')

		rospy.spin()
	   
	except rospy.ROSInterruptException:
		pass

if __name__ == '__main__':
	main()