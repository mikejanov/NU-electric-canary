#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import monti_msgs.msg as msgs
import numpy
import serial
from serial import SerialException
from array import array

class montiRobot():
	def __init__(self, name = 'monti/'):
		rospy.init_node('monti')
		rospy.loginfo("Starting the monti_robot ROS node")
		self.rate = rospy.Rate(15)
		self.ser = serial.Serial()

		#Message Headers
		self.command_header = 0x02
		self.config_header = 0x01
		
		#self.sensors = [["BME280", 3, "Temperature", "Pressure", "Humidity"], ["LIS3DH", 3, "Accelerometer X", "Accelerometer Y", "Accelerometer Z"], ["US5881LUA", 1, "Hall Effect"], ["MiCS5524", 1, "CO, Alcohol, and VOC Gas"]]
		self.pods = []
		self.pods.append({})
		self.pods.append({'pod_id':'\x01', "part_number":"BME280", 'present_data_types':["Temperature", "Pressure", "Humidity"], 'bytes_per_data_type':[4, 4, 4]})
		self.pods.append({'pod_id':'\x02', "part_number":"LIS3DH", 'present_data_types':["Accelerometer X", "Accelerometer Y", "Accelerometer Z"], 'bytes_per_data_type':[2, 2, 2]})
		self.pods.append({'pod_id':'\x03', "part_number":"US5881LUA", 'present_data_types':["Hall Effect"], 'bytes_per_data_type':[1]})
		self.pods.append({'pod_id':'\x04', "part_number":"MiCS5524", 'present_data_types':["CO, Alcohol, and VOC Gas"], 'bytes_per_data_type':[1]})

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

		#Poll Monti State Messages
		# self.poll_monti()

	def init_comms_cb(self, state):
		if (state.data == "connect"): # Open serial comms with monti
			serial_ports = ['/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyUSB2', '/dev/ttyUSB3', '/dev/ttyUSB4', '/dev/ttyUSB5' ]
			#Initialize communication with the Monti ROV
			for serial_port in serial_ports:
				try:
					self.ser = serial.Serial(serial_port, baudrate=9600, parity='E', bytesize=7)
					#self.ser.write('MontiPython')
					rospy.loginfo("Successfully connected to the Monti ROV on serial port " + serial_port)
					return # Once successfully connected, stop trying
				except SerialException:
					rospy.loginfo("Unable to connect to Monti on port " + serial_port)

			# while (1):
			# 	words = ser.read(11)
			# 	print(words)
			
			#Poll Monti State Messages
		elif (state.data == "disconnect"): # Close serial comms with monti
			print ("Disconnecting from Monti")
			self.ser.close()

	def poll_monti(self):
		# Read state messages from the serial port while it is open and data is present
		rospy.loginfo("Hello! Trying to conenct!")
		while (not rospy.is_shutdown()):
			if (self.ser.isOpen()):
				rospy.loginfo("Hello! open serial")
				if (self.ser.inWaiting()>0):
					rospy.loginfo("Hello! reading serial")
					raw_data = self.ser.read(32)
					self.pub_monti_state(raw_data)

	def set_robot_config(self):
		self.present_sensors = input("Please enter the ID numbers of present modules: ")

		self.msg_config = [] #Stores the starting sensor msg index of each sensor
		self.packet_config = [] #Stores the starting packet index of each sensor
		packet_pos = 1
		msg_pos = 0
		for sensor_id in self.present_sensors:
			#Save the start position for the sensor data in the state message
			self.packet_config.append(packet_pos)
			self.msg_config.append(msg_pos)

			#Calculate where the next sensor starts in the state packet
			for data_type in range(len(self.pods[sensor_id]['present_data_types'])):
				packet_pos = packet_pos + self.pods[sensor_id]['bytes_per_data_type'][data_type]
			
			#Calculate where the next sensor starts in the state message
			msg_pos = msg_pos + len(self.pods[sensor_id]['present_data_types'])

	def pub_monti_state(self, data):
		# Function called when data is received from robot
		# Publishes state data to the state topic
		state = msgs.Monti_Robot_State()

		state.header = 0xFF
		state.error = 12
		for i in range(6):
			state.encoders[i] = 24

		if (len(self.present_sensors)>0):
			p=0
			for sensor_id in self.present_sensors:  #For each present pod
				pod_msg_start = self.msg_config[p]
				for i in range(len(self.pods[sensor_id]['present_data_types'])):  #For each type of data recorded in the given pod
					# print(pod_msg_start+i)
					state.modular_sensors[pod_msg_start + i].part_number = self.pods[sensor_id]['part_number']
					state.modular_sensors[pod_msg_start + i].type_of_data = self.pods[sensor_id]['present_data_types'][i]
					state.modular_sensors[pod_msg_start + i].data = i
				p=p+1

		self.state_pub.publish(state)
		self.rate.sleep()

	def send_monti_control_cb(self, command):
		#Send the control command to the Monti ROV
		rospy.loginfo("Sending motion control command to Monti")
		print ("Moving in direction " + str(command.direction) + " at throttle " + str(command.throttle) + " for " + str(command.actuation_time) + " seconds")
		t = command.throttle*25
		control_buff = [self.command_header, command.direction, t, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
		self.write_to_monti(control_buff) 
		#self.ser.write()

	def send_monti_config_cb(self, command):
		#Send the control command to the Monti ROV
		rospy.loginfo("Sending configuration to Monti")
		#self.ser.write()

	def write_to_monti(self, buffer):
		if self.ser.isOpen():
			packet = array('b', buffer).tostring()
			self.ser.write(packet)

def main():
	montiRobot()

if __name__=='__main__':
	main()