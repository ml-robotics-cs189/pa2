import rosbag
import parse_bagfiles
import numpy as np
import matplotlib.pyplot as plt
import GPy

class GP:

	def __init__(self):
		

	def readmsg(self):
		bagfile = '/home/cs89/catkin_ws/src/team1/bagfiles/random.bag'
		sensor_topic = '/kf1/simulated_sensor/raw'

		i = 0 	# iterator

    # water quality data
		sensor_lat = np.zeros([sensor_msg_cnt])
		sensor_long = np.zeros([sensor_msg_cnt])
		sensor_data = np.zeros([sensor_msg_cnt])
	#sensor_type = np.zeros([sensor_msg_cnt])

	# loop over the topic to read evey message
		for topic, msg, t in bag.read_messages(topics=sensor_topic):
			sensor_lat[i] = msg.latitude
			sensor_long[i] = msg.longitude
			sensor_data[i] = msg.data
		#sensor_type[i] = msg.type
			i += 1

		gps_data = []
		for i in range(0, sensor_msg_cnt):
			item = sensor_lat[i], sensor_long[i]
			gps_data.append(tuple(item))

		print("Sensor: " , sensor_data)
		print("GPS: ", gps_data)

		bag.close()

if __name__ == "__main__":
	gp = GP()
	
	



