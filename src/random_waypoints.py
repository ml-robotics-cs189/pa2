#! /usr/bin/python
import rospy
import random
from gps_nav.srv import Goto
from std_msgs.msg import Int8

# for checking bounds
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon


if __name__ == '__main__':
	rospy.init_node('navigation')

	#### CONFIGURE THE PARAMETERS BELOW ####
	n = 20 # number of total waypoints desired
	corner_pts = ([33.44481,-118.48498],[33.44443,-118.48435],[33.44469,-118.48413],[33.44507,-118.48474]) # Coordinates of bounding region
	########################################

	i = 1	# counter for number of waypoints visited
	r = rospy.Rate(10)	 # frequency 
	polygon = Polygon(corner_pts) # create polygon
	
	# services
	go_to = rospy.ServiceProxy('/kf1/goto', Goto)


	while not rospy.is_shutdown():
		if rospy.is_shutdown():
			print('shutdown')
			break

		if i > n:
			rospy.is_shutdown()
		else:
			try:
				rospy.wait_for_service('/kf1/goto') 	# ensure that the service is available 

				# generate random coordinate within bounds
				latitude = float(random.uniform(33.44443, 33.44507))
				longitude = float(random.uniform(-118.48413, -118.48498))

				point = Point(latitude,longitude) # create point

				if point.within(polygon): # check if a point is in the polygon 
					out = go_to(latitude, longitude)
					if out.result == True:
						rospy.loginfo("going to coordinate " + str(i) + ":")
						rospy.loginfo(point)
						i = i + 1

						# sleep until waypoint has been reached
						try:
							rospy.wait_for_message('/kf1/waypoint_goto_result', Int8, 500)
						except:
							rospy.loginfo("timeout waiting for robot to go to coordinate")

					else:
						rospy.loginfo("Failed to go to coordinate")

			except rospy.ServiceException, e:
				rospy.loginfo("Service call failed")