import rospy

class montiRobot():
	def __init__(self):
		rospy.init_node('monti_robot')
		rospy.loginfo("Starting the monti_robot ROS node")

def main():
	montiRobot()

if __name__=='__main__':
	main()