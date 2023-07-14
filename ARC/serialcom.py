#!/usr/bin/env python
import roslib
import rospy
from sensor.msg import Rover_sensor
import serial
import json
import time

ser = serial.Serial('/dev/ttyUSB0', 115200)
rospy.set_param('motorState', 0)

def sensor():
	while not rospy.is_shutdown():
		data = ser.readline().decode(encoding='utf-8', errors='ignore')
		try:
			j = json.loads(data)
			msg = Rover_sensor()
			msg.temperature = j['temp']
			msg.humidity = j['humidity']
			#msg.pressure = j['pressure'] 
			#msg.altitude = j['altitude'] 
			msg.met_gas_val = j['met_gas_val'] 
			msg.weight_average = j["weight_average"]

			msg.header.stamp = rospy.Time.now()
			rospy.loginfo(msg)
			pub.publish(msg)
			rospy.sleep(.2)
			print(data)
			get_control_data()
		except json.JSONDecodeError as e:
				print("JSON:", e)
def get_control_data():
	
	motorState = rospy.get_param('motorState');
	"""	
 	motorFlag = rospy.get_param('motorFlag');
 	tube_to_go = rospy.get_param('motorToGo');
	valve1 = rospy.get_param('valve1');
	valve2 = rospy.get_param('valve2');
	valve3 = rospy.get_param('valve3');
	valve4 = rospy.get_param('valve4');
	pump = rospy.get_param('pump');
	received_states = [ valve1, valve2, valve3, valve4, pump]
	data = {'motorFlag': motorFlag, 'tube_to_go': tube_to_go,'received_states':received_states}
	"""
	data = {'motorState': motorState}

	# Convert the dictionary to a JSON string
	json_data = json.dumps(data)

	# Convert the JSON string to bytes
	bytes_data = json_data.encode('utf-8')

	# Send the bytes data over the serial port
	ser.write(bytes_data)
	time.sleep(0.1) # wait 0.1 second
	

if __name__ == '__main__':
	try:
		pub = rospy.Publisher('sensor_raw', Rover_sensor, queue_size=10) # TODO: change message type here
		rospy.init_node('sensor')
		sensor()
		
	except rospy.ROSInterruptException:
		pass
