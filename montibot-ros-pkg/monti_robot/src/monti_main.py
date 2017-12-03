#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import monti_msgs.msg as msgs
import numpy
import serial

class montiRobot():
	def __init__(self, name = 'monti/'):
		rospy.init_node('monti')
		rospy.loginfo("Starting the monti_robot ROS node")
		self.rate = rospy.Rate(15)
		
		self.sensors = [["BME280", 3, "Temperature", "Pressure", "Humidity"], ["LIS3DH", 3, "Accelerometer X", "Accelerometer Y", "Accelerometer Z"], ["US5881LUA", 1, "Hall Effect"], ["MiCS5524", 1, "CO, Alcohol, and VOC Gas"]]

		# Create the robot state publisher
		self.state_pub = rospy.Publisher(name + 'state', msgs.Monti_Robot_State, queue_size = 10)
		
		#Subscribe to the command and initiallization topics
		rospy.Subscriber(name + 'connection_status', String, self.init_comms_cb)
		rospy.Subscriber(name + 'control_command', msgs.Monti_Control, self.send_monti_control_cb)
		rospy.Subscriber(name + 'config', msgs.Monti_Config, self.send_monti_config_cb)

		#self.init_comms()

		#set an initital configuration state containing only the drivetrain
		self.set_robot_config()

		#Tesing the messages
		while not rospy.is_shutdown():
			self.pub_monti_state(5)

	def init_comms_cb(self, state):
		if (state.data == "connect"):
			print ("initialized connection woooooh!")
			# #Initialize communication with the Monti ROV
			# ser = serial.Serial('/dev/ttyUSB1', baudrate=9600, parity='E', bytesize=7)
			# ser.write('MontiPython')
			# rospy.loginfo("Successfully connected to the Monti ROV on port A")
			# words = ser.read(11)
		else:
			print ("Failed to connect!")
			ser.close()


	def set_robot_config(self):
		self.present_sensors = input("Please enter the ID numbers of present modules: ")

		self.packet_config = [] #Stores the starting packet index of each sensor
		packet_pos = 0
		for sensor_id in self.present_sensors:
			#Save the start position for the sensor data in the state message
			self.packet_config.append(packet_pos)

			#Calculate where the next sensor starts in the state message
			packet_pos = packet_pos + self.sensors[sensor_id][1]

	def pub_monti_state(self, state):
		# Function called when data is received from robot
		# Publishes state data to the state topic
		state = msgs.Monti_Robot_State()

		state.header = 0xFF
		state.error = 12
		for i in range(6):
			state.encoders[i] = 24

		if (len(self.present_sensors)>0):
			p=0
			for sensor_id in self.present_sensors:
				sensor_start = self.packet_config[p]
				for i in range(self.sensors[sensor_id][1]):
					state.modular_sensors[sensor_start + i].part_number = self.sensors[sensor_id][0]
					state.modular_sensors[sensor_start + i].type_of_data = self.sensors[sensor_id][i+2]
					state.modular_sensors[sensor_start + i].data = i
				p=p+1

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