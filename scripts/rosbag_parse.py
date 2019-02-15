import rosbag
import numpy as np
import matplotlib.pyplot as plt 
import gp
import json


# uncomment if you have a bagfile_location.json file
bagfiles = []
try:
	with open('bagfile_location.json') as f:
		data = json.load(f)
		bagfiles = data["bagfile_paths"]
except IOError:
	print("Please create a JSON file containing the location of your " +
		"bag file. \nCreate a dictionary, set the key \"bagfile_paths\" equal " +
		"to the array of paths, and save to a file named \"bagfile_location.json\".")

# uncomment if you have the location of each bagfile path
# bagfile = ['/home/cs89/catkin_ws/src/team1/bagfiles/random.bag']


sensor_topic = '/kf1/simulated_sensor/raw'



# read messages
def readmsg(file):
	bag = rosbag.Bag(file)
	sensor_msg_cnt = bag.get_message_count(sensor_topic)

	i = 0 	# iterator

    # water quality data
	sensor_lat = np.zeros([sensor_msg_cnt])
	sensor_long = np.zeros([sensor_msg_cnt])
	sensor_data = np.zeros([sensor_msg_cnt])

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

	bag.close()


def parseBag(bag, file_num):
	readBag(bag)
	plotBag(gps_data, sensor_data, True, file_num)


def readBag(bag):
	# read the bagfile
	i = 0 	# iterator
	for topic, msg, t in bag.read_messages(topics=sensor_topic):
		sensor_data[i] = [ msg.data ]
		gps_data[i] = [ msg.latitude, msg.longitude ]

		i += 1

	print("finished reading bag file")

	bag.close()


def plotBag(gps_data, sensor_data, save_to_file=False, file_num = 0):

	fig = plt.figure()
	ax = fig.add_subplot(111, projection='3d')
	ax.scatter(gps_data[:,0], gps_data[:,1], sensor_data)

	if save_to_file:
		plt.savefig("sensor_data_" + str(file_num) + ".pdf")
	else:
		plt.show()


if __name__ == "__main__":
	file_num = 0
	for file in bagfiles:
		bag = rosbag.Bag(file)

		sensor_msg_cnt = bag.get_message_count(sensor_topic)

		# water quality data
		gps_data = np.zeros((sensor_msg_cnt, 2))
		sensor_data = np.zeros((sensor_msg_cnt, 1))

		parseBag(bag, file_num)
		file_num += 1
  
	
# store data in arrays
