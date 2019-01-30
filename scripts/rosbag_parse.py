import rosbag
import numpy as np
import matplotlib.pyplot as plt 


######
# Did: 
	# Basic outline on grabbing the simulated water quality sensor from the rosbag file 
	# Some data grabbing of the sensor data
# Todo: 
	# Plotting the boundary still needs to be done
# Issues:
	# Is this the right data? 
######

bagfile = '../bagfiles/random_waypoints_2019-01-29-23-18-41.bag'
sensor_topic = '/kf1/simulated_sensor/raw'


# read messages
def readmsg():
	bag = rosbag.Bag(bagfile)
	sensor_msg_cnt = bag.get_message_count(sensor_topic)

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

	print(sensor_data)

	bag.close()


if __name__ == "__main__":
	readmsg()

	# store data in arrays
