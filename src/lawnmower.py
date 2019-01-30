#! /usr/bin/python
import rospy
from shapely.geometry import Polygon
import numpy as np
from helpers import distance, min_and_max_y
from waypoint_generation import generate_waypoints_updown

from sensor_msgs.msg import NavSatFix
from gps_nav.srv import Goto
from std_msgs.msg import Int8

class lawnmower_search:
	def __init__(self):

		self.cur_loc = rospy.Subscriber("/kf1/navsat/fix", NavSatFix, self.callback)
		self.goto = rospy.ServiceProxy('/kf1/goto', Goto)
		self.lat = None
		self.long = None

		print("Here")

		rospy.wait_for_message("/kf1/navsat/fix", NavSatFix)
		

	def callback(self, cur_loc):
		print(cur_loc.latitude)
		self.lat = cur_loc.latitude
		self.long = cur_loc.longitude

	def controller(self, region, waypoints, approach_distance):
		""" 
			get_location: returns the XY location of the boat
			set_target_location: causes the boat to head towards the given (X, Y)
			approach_distance: how close the boat must approach
				a waypoint in order to have "reached" it & move on
		"""

		#waypoints = generate_waypoints(region)
	
	# treat the waypoints as a stack, popping off the top one
	# and heading towards it until none remain

		print("Controller")

		waypoints.reverse()
		while len(waypoints) > 0:
			target = waypoints.pop()
			print("Target", target)

			try:
				rospy.wait_for_service("/kf1/goto")

				out = self.goto(target[0],target[1])
				if out.result == True:
					#rospy.loginfo("going to coordinate " + target)


							# sleep until message recieved
					try:
						rospy.wait_for_message('/kf1/waypoint_goto_result', Int8, 500)
					except:
						rospy.loginfo("timeout waiting for robot to go to coordinate")

				else:
					rospy.loginfo("Failed to go to coordinate")

			except rospy.ServiceException, e:
				rospy.loginfo("Service call failed")



	def get_catalina_region(self):
		normalized = False
		print("Catalina region")

		catalina = [(33.44481,-118.48498),(33.44443,-118.48435),(33.44469,-118.48413),(33.44507,-118.48474)]
		if normalized:
			c_fixed = []
			for x, y in catalina:
				x_new = ((x * 10000) % 100 - 44) * 50
				y_new = ((y * 10000) % 100 - 50) * 50
				c_fixed.append((x_new, y_new))
			catalina = c_fixed

		return Polygon(catalina)


if __name__ == "__main__":
	rospy.init_node('lawnmower_search', anonymous = True)
	test = lawnmower_search()
	print("Here2")
	region = test.get_catalina_region()
	waypoints = generate_waypoints_updown(region)
	test.controller(region, waypoints, approach_distance = 0.00001)
	while not rospy.is_shutdown():
		rospy.spin()
	
