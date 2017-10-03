#!/usr/bin/env python
import rospy
import monti_msgs.msg as msgs

class montiRobot():
	def __init__(self, name = 'monti_main/'):
		rospy.init_node('monti_main')
		rospy.loginfo("Starting the monti_robot ROS node")
		self.rate = rospy.Rate(15)

		# Create the robot state publisher
		self.state_pub = rospy.Publisher(name + 'state', msgs.Monti_Robot_State, queue_size = 10)
		for x in range(4000000):
			for p in range(200):
				self.pub_monti_state(p)

	def pub_monti_state(self,n):
		# Function called when data is reveived from robot
		# Publishes state data to the state topic
		state = msgs.Monti_Robot_State()

		state.header = 0xFF
		state.error = 17
		for i in range(6):
			state.encoders[i] = i

		state.accelerometer.triple_axis_accel = 12
		state.accelerometer.x_accel = 4
		state.accelerometer.y_accel = 17
		state.accelerometer.z_accel = 10

		state.environment.pressure = 4
		state.environment.temperature = 8
		state.environment.humidity = 12

		state.temperature = 17

		for i in range(10):
			state.misc_sensors[i] = i

		self.state_pub.publish(state)
		self.rate.sleep()

def main():
	montiRobot()

if __name__=='__main__':
	main()