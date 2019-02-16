import rosbag
import rospy
import numpy as np
import gp
import json
from sensor_msgs.msg import NavSatFix
from simulated_sensor.msg import Measurement
import random

from shapely.geometry import Point
from shapely.geometry.polygon import Polygon

from std_msgs.msg import Int8
from gps_nav.srv import Goto

from collections import deque


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

		# variable for sensor topic
		self.sensor_topic = '/kf1/simulated_sensor/raw'
		self.counter = 0

		print("Reading bagfile \"" + self.bagfile + "\"")
		self.bag = rosbag.Bag(self.bagfile)

		sensor_msg_cnt = self.bag.get_message_count(self.sensor_topic)
		print("Read %d messages from the bagfile. " % sensor_msg_cnt)

		# initialize arrays to contain gps_data and sensor_data

		self.gps_data = np.zeros((sensor_msg_cnt, 2))
		self.sensor_data = np.zeros((sensor_msg_cnt, 1))


		# initialize variables
		self.cur_data = None
		self.cur_data_lat = None
		self.cur_data_long = None

		self.gaussian = gp.GP()

		# subscribers
		self.sensor = rospy.Subscriber("/kf1/simulated_sensor/raw", Measurement, self.callback)
		#self.goto = rospy.Subscriber("/kf1/waypoint_goto_result", Int8, self.goto_hotspot)

		#self.goto = rospy.Subscriber("/kf1/waypoint_goto_result", Int8, self.callback_chloro)
		print("after callback")

		rospy.wait_for_message("/kf1/simulated_sensor/raw", Measurement)
		#rospy.wait_for_message("/kf1/waypoint_goto_result", Int8)

		print("received message")


		self.go_to = rospy.ServiceProxy('/kf1/goto', Goto)

		#self.set_waypoint(self.go_to)
		self.controller()


	def callback(self, sensor):
		'''
		consistantly add new data to global data array
		'''		
		self.cur_data = sensor.data
		self.cur_data_lat = sensor.latitude
		self.cur_data_long = sensor.longitude

		#print(self.cur_data_lat, self.cur_data_long)

		# adding current data readings to arrays
		lat_long_data = [ self.cur_data_lat, self.cur_data_long ]

		self.gps_data = np.vstack([self.gps_data, lat_long_data])
		self.sensor_data = np.vstack([self.sensor_data, [self.cur_data]])

		#self.gaussian.add_data(self.gps_data, self.sensor_data)
		


	def controller(self):

		corner_pts = ([33.44481,-118.48498],[33.44443,-118.48435],[33.44469,-118.48413],[33.44507,-118.48474]) # Coordinates of bounding region
		polygon = Polygon(corner_pts)

		q = deque()


		q.append([33.44843, -118.484869])
		q.append([33.45002, -118.484746])
		q.append([33.444783, -118.484674])
		q.append([33.444914, -118.484574])
		q.append([33.444683, -118.484541])
		q.append([33.444815, -118.484439])
		q.append([33.444595, -118.484372])
		q.append([33.444730, -118.484257])

		while(len(q) != 0):


			print("pop")
			next = q.pop()

			latitude = next[0]
			longitude = next[1]

			point = Point(latitude, longitude)

			if(point.within(polygon)):

				self.set_waypoint(self.go_to, latitude, longitude)
			

				self.gaussian.add_data(self.gps_data, self.sensor_data)
				predictive = self.gaussian.gaussian_proc(np.array([[self.cur_data_lat, self.cur_data_long]]))

				print(predictive)

				latitude = predictive[0][0]
				longitude = predictive[0][1]

				point = Point(latitude, longitude)

				

				if(abs(self.cur_data_lat - latitude) > 0.0001 or abs(self.cur_data_long - longitude) > 0.0001):

					if(point.within(polygon)):
							#point = Point(latitude, longitude)

						#if(point.within(polygon)):

						self.set_waypoint(self.go_to, latitude, longitude)

						self.gaussian.add_data(self.gps_data, self.sensor_data)
						predictive = self.gaussian.gaussian_proc(np.array([[self.cur_data_lat, self.cur_data_long]]))

						latitude = predictive[0][0]
						longitude = predictive[0][1]

				#go back to gaussian


		print("done")



	def set_waypoint(self, go_to, latitude, longitude):


		try:

			rospy.wait_for_service('/kf1/goto')

			print("go service")

			#next = q.get()

			#latitude = next[0]
			#longitude = next[1]

			point = Point(latitude, longitude)


			out = go_to(latitude, longitude)

			if out.result == True:
					#rospy.loginfo("going to coordinate " + st+ ":")
				rospy.loginfo(point)
					#print(self.cur_data_lat, self.cur_data_long)
					#i = i + 1

					# sleep until waypoint has been reached
				try:
					rospy.wait_for_message('/kf1/waypoint_goto_result', Int8, 500)
						

				except:
					rospy.loginfo("timeout waiting for robot to go to coordinate")

			else:
				rospy.loginfo("Failed to go to coordinate")

		except rospy.ServiceException, e:
			rospy.loginfo("Service call failed")


	def goto_waypoint(self, go_to):

		print("here")

		#NEED TO ADD DATA WHILE TRAVELING
		corner_pts = ([33.44481,-118.48498],[33.44443,-118.48435],[33.44469,-118.48413],[33.44507,-118.48474]) # Coordinates of bounding region
		polygon = Polygon(corner_pts)

		print("after polygron")

		try:

			rospy.wait_for_service('/kf1/goto')

			print("go service")

			latitude = float(random.uniform(33.44443, 33.44507))
			longitude = float(random.uniform(-118.48413, -118.48498))

			point = Point(latitude,longitude)

			print("after point")

			#if point.within(polygon): # check if a point is in the polygon 
			print("point in polygon")
			out = go_to(latitude, longitude)
			print("got out")
			if out.result == True:
					#rospy.loginfo("going to coordinate " + st+ ":")
				rospy.loginfo(point)
					#print(self.cur_data_lat, self.cur_data_long)
					#i = i + 1

					# sleep until waypoint has been reached
				try:
					rospy.wait_for_message('/kf1/waypoint_goto_result', Int8, 500)
						

				except:
					rospy.loginfo("timeout waiting for robot to go to coordinate")

			else:
				rospy.loginfo("Failed to go to coordinate")

		except rospy.ServiceException, e:
			rospy.loginfo("Service call failed")

		#self.gaussian.gaussian_proc([la])



	def show_sensor_data(self, save_to_file=False):

		fig = plt.figure()
		ax = fig.add_subplot(111, projection='3d')
		ax.scatter(self.gps_data[:,0], self.gps_data[:,1], self.sensor_data)
		if save_to_file:
			plt.savefig("sensor_data.pdf")
		else:
			plt.show()
	


if __name__ == "__main__":
	
	rospy.init_node('gp', anonymous=True)

	r = rospy.Rate(10) 

	#initial ros node class
	n = node()	

	#set first waypoint randoml

	while not rospy.is_shutdown():
		rospy.spin()
	


	#data_sub = ()

