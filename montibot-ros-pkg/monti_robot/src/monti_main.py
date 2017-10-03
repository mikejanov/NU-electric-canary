import rospy
import monti_msgs as msgs

class montiRobot():
	def __init__(self, name = 'monti_main/'):
		rospy.init_node('monti_main')
		rospy.loginfo("Starting the monti_robot ROS node")

		# Create the robot state publisher
		state_pub = rospy.Publisher(name + 'state', msgs.Monti_Robot_State, queue_size = 10)

	def pub_monti_state(self):
		# Function called when data is reveived from robot
		# Publishes state data to the state topic
		state = msgs.Monti_Robot_State()


def main():
	montiRobot()

if __name__=='__main__':
	main()