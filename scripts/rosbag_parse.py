import rosbag
import numpy as np
import matplotlib.pyplot as plt 


######
# Did: 
	# Basic outline on grabbing the gps coordinates from the rosbag file 
	# Some data grabbing of the sensor data
# Todo: 
	# Make sure that sensor data is being collected properly
	# plotting the boundary still needs to be done
# Issues: 
	# Handle case with different number of messagse
	# not sure if time is being computed properly
######

bagfile = '../bagfiles/random_waypoints_2019-01-29-17-29-36.bag'
gps_coords = '/kf1/navsat/fix'
sensor_data = '/kf1/imu/data'


# read messages
def readmsg(topic):
	print("here")
	bag = rosbag.Bag(bagfile)
	gps_msg_cnt = bag.get_message_count(gps_coords)
	sensor_msg_cnt = bag.get_message_count(sensor_data)

	i = 0 	# iterator
	# gps data
	gps_lat = np.zeros([gps_msg_cnt])
	gps_lon = np.zeros([gps_msg_cnt])

	# sensor data - might need more 
	sensor_data1 = np.zeros([sensor_msg_cnt])
	sensor_data2 = np.zeros([sensor_msg_cnt])
	sensor_data3 = np.zeros([sensor_msg_cnt])
	sensor_data4 = np.zeros([sensor_msg_cnt])

	# might have to check whether sensor_msg_cnt is larger than gps_msg_cnt
	time = np.zeros([sensor_msg_cnt])

	# loop over the topic to read evey message
	for topic, msg, t in bag.read_messages(topics=topic):
		sec         = t.to_nsec() 
		time[i]  = (sec-1508618888979416609)*1e-9
		gps_lat[i] = msg.latitude
		gps_lon[i] = msg.longitude
		sensor_data1[i] = msg.orientation.x
		sensor_data2[i] = msg.orientation.y
		sensor_data3[i] = msg.orientation.z
		sensor_data4[i] = msg.orientation.w
		i += 1


	bag.close()


def main():
	readmsg(gps_coords)

	# store data in arrays

main()

