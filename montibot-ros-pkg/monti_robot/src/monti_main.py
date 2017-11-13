#!/usr/bin/env python
import rospy
import monti_msgs.msg as msgs

class montiRobot():
	def __init__(self, name = 'monti/'):
		rospy.init_node('monti')
		rospy.loginfo("Starting the monti_robot ROS node")
		self.rate = rospy.Rate(15)
		
		self.sensors = [["BNO055", "IMU"], ["US5881LUA", "Hall Effect"], ["MCP9808", "Temperature"]]

		# Create the robot state publisher
		self.state_pub = rospy.Publisher(name + 'state', msgs.Monti_Robot_State, queue_size = 10)
		
		#Subscribe to the command and initiallization topics
		rospy.Subscriber(name + 'control_command', msgs.Monti_Control, self.send_monti_control_cb)
		rospy.Subscriber(name + 'config', msgs.Monti_Config, self.send_monti_config_cb)

		self.set_robot_config()
		while not rospy.is_shutdown():
			self.pub_monti_state(5)

	def init_comms(self):
		#Initialize communication with the Monti ROV
		rospy.loginfo("Successfully connected to the Monti ROV on port A")

	def set_robot_config(self):
		self.present_sensors = input("Please enter the ID numbers of present modules: ")

	def pub_monti_state(self, state):
		# Function called when data is reveived from robot
		# Publishes state data to the state topic
		state = msgs.Monti_Robot_State()

		state.header = 0xFF
		state.error = 12
		for i in range(6):
			state.encoders[i] = 24

		state.accelerometer.triple_axis_accel = 12
		state.accelerometer.x_accel = 4
		state.accelerometer.y_accel = 17
		state.accelerometer.z_accel = 10

		state.environment.pressure = 4
		state.environment.temperature = 8
		state.environment.humidity = 12

		state.temperature = 17

		if (len(self.present_sensors)>0):
			for i in range(len(self.present_sensors)):
				state.modular_sensors[i].part_number = self.sensors[self.present_sensors[i]][0]
				state.modular_sensors[i].type_of_data = self.sensors[self.present_sensors[i]][1]
				state.modular_sensors[i].data = i

		self.state_pub.publish(state)
		self.rate.sleep()

	def send_monti_control_cb(self, command):
		#Send the control command to the Monti ROV
		rospy.loginfo("Sending motion control command to Monti")

	def send_monti_config_cb(self, command):
		#Send the control command to the Monti ROV
		rospy.loginfo("Sending configuration to Monti")

def main():
	montiRobot()

if __name__=='__main__':
	main()