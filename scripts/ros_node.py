import rosbag
import rospy
import numpy as np
import gp
import json
from sensor_msgs.msg import NavSatFix
from simulated_sensor.msg import Measurement

from scipy.cluster.vq import kmeans

class node:

	def __init__(self):

		# read the bag file LOCATION from a json file:
		try:
			with open('bagfile_location.json') as f:
				data = json.load(f)
				self.bagfile = data["bagfile_path"]
		except IOError:
			print("Please create a JSON file containing the location of your " +
				"bag file. \nCreate a dictionary, set the key \"bagfile_path\" equal " +
				"to the path, and save to a file named \"bagfile_location.json\".")
			return

		#self.bagfile = '/home/cs89/catkin_ws/src/team1/pa2/bagfiles/lawnmower.bag'

		self.sensor_topic = '/kf1/simulated_sensor/raw'

		print("Reading bagfile \"" + self.bagfile + "\"")
		self.bag = rosbag.Bag(self.bagfile)

		sensor_msg_cnt = self.bag.get_message_count(self.sensor_topic)
		print("Read %d messages from the bagfile. " % sensor_msg_cnt)

		self.gps_data = np.zeros((sensor_msg_cnt, 2))
		self.sensor_data = np.zeros((sensor_msg_cnt, 1))

		self.readbag()

		self.sensor = rospy.Subscriber("/kf1/simulated_sensor/raw", Measurement, self.callback)

		#rospy.wait_for_message("/kf1/simulated_sensor/raw", Measurement)

		self.cur_data = None
		self.cur_data_lat = None
		self.cur_data_long = None

		#self.gaussian_proc()

	def callback(self, sensor):

		print("here")
		self.cur_data = sensor.data
		self.cur_data_lat = sensor.latitude
		self.cur_data_long = sensor.longitude

		#need to add current data readings to what was read from bagfile
		#gps_data and sensor_data contain info from bagfile

		lat_long_data = [ self.cur_data_lat, self.cur_data_long ]

		self.gps_data = np.vstack((self.gps_data, lat_long_data))
		self.sensor_data = np.vstack((self.sensor_data, self.cur_data))


	def readbag(self):
		i = 0 	# iterator
		for topic, msg, t in self.bag.read_messages(topics=self.sensor_topic):
			self.sensor_data[i] = [ msg.data ]
			self.gps_data[i] = [ msg.latitude, msg.longitude ]

			i += 1

		print("finsihed reading bag file")

		self.bag.close()


	def show_sensor_data(self, save_to_file=False):

		fig = plt.figure()
		ax = fig.add_subplot(111, projection='3d')
		ax.scatter(self.gps_data[:,0], self.gps_data[:,1], self.sensor_data)
		if save_to_file:
			plt.savefig("sensor_data.pdf")
		else:
			plt.show()





if __name__ == "__main__":

	#rospy.init_node('gp_movement', anonymous=True)

	n = node()

	print("after node")

	process = gp.GP(n.gps_data, n.sensor_data)
	print("after gp")
	#model = process.gaussian_proc()

	while not rospy.is_shutdown():
		rospy.spin()



	#data_sub = ()
