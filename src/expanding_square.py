#!/usr/bin/python

# Expanding square sweep method

import rospy
import math
from tf.transformations import euler_from_quaternion
from gps_nav.srv import Goto # send latitude and longitude coordinates
from sensor_msgs.msg import NavSatFix, Imu # receive current robot coordinates
from std_msgs.msg import Int8 # result if the robot reached the destination from goto
from usv_gazebo_plugins.msg import UsvDrive # send left and right motor commands
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon


# helper angle functions
def rectify_angle_pi(angle):
	angle = rectify_angle_2pi(angle)
	if(angle > math.pi):
		angle -= 2 * math.pi
	return angle


def rectify_angle_2pi(angle):
	while(angle < 0):
		angle += 2 * math.pi
	while(angle > 2 * math.pi):
		angle -= 2 * math.pi
	return angle


class expanding_square:
	def __init__(self):
		self.boundary = ([33.44481,-118.48498],[33.44443,-118.48435],[33.44469,-118.48413],[33.44507,-118.48474])
		self.polygon = Polygon(self.boundary)

		self.angle = None
		self.angle_thresh = 0.2
		self.target_angle = 0.0
		
		self.location = None
		self.dest = None
		
		self.iter = 2
		self.n = 1
		self.travel_dist = 0.00004 		# unit travel
		self.travel = self.travel_dist * self.n 	# distance to travel for the current robot's state
		self.driving = True


		# want updates of the robot location and orientation
		self.navsat_sub = rospy.Subscriber("/kf1/navsat/fix", NavSatFix, self.navsat_callback)
		self.imu_sub = rospy.Subscriber("/kf1/imu/data", Imu, self.imu_callback)

		# subscribe to topics and services that controls the robot
		self.go_to = rospy.ServiceProxy("/kf1/goto", Goto)
		self.cmd_drive = rospy.Publisher("/kf1/cmd_drive", UsvDrive, queue_size = 0)

		# wait for messages to come through before beginging sweep method
		rospy.wait_for_message("/kf1/navsat/fix", NavSatFix)
		rospy.wait_for_message("kf1/imu/data", Imu)

		self.turn_to_initial_pos()


	# callback function for collecting updates on the robot's longitude and latitude position
	def navsat_callback(self, navsat_data):
		# location is the current location of the robot
		self.location = Point(navsat_data.latitude, navsat_data.longitude)

		if(self.dest == None): # start as the original position of the robot
			self.dest = Point(navsat_data.latitude, navsat_data.longitude)
			rospy.loginfo("start location: " + str(self.dest))


	# callback function for collecting updates on the robot's orientation, specifically the yaw
	def imu_callback(self, imu_data):
		orientation = (
			imu_data.orientation.x,
			imu_data.orientation.y,
			imu_data.orientation.z,
			imu_data.orientation.w
		)
		(roll, pitch, yaw) = euler_from_quaternion(orientation)
		self.angle = yaw


	# function to orient the robot in the direction perpendicular to the edge of one of the boundaries
	def turn_to_initial_pos(self):

		# rotating the robot until close to the threshold
		while abs(rectify_angle_pi(self.angle - self.target_angle)) >= self.angle_thresh:
			self.cmd_drive.publish(0.1, -0.1) # left motor; right motor
		# robot's orientation still needs to be changed
		else:
			self.cmd_drive.publish(0.0, 0.0) # left motor; right motor



	# expanding square search method that calculates the next destination point and proceeds to go there
	def search(self):
		if(self.driving):
			try:

				# DESTINATION CALCULATION: calculate in which direction to go
				if(not self.iter % 2): # longitude
					if( (self.iter % 4) == 2 ): # moving west
						print("driving west")
						self.dest = Point(self.dest.x, self.dest.y - self.travel)
					else: 						# moving east
						print("driving east")
						self.dest = Point(self.dest.x, self.dest.y + self.travel)
				else: # latitude
					if( (self.iter % 4) == 1 ): # moving north
						print("driving north")
						self.dest = Point(self.dest.x + self.travel, self.dest.y)
					else: 						# moving south
						print("driving south")
						self.dest = Point(self.dest.x - self.travel, self.dest.y)

				if(self.iter % 2):	# for every two travels, increase the distance of travel
					self.n += 1
					self.travel = self.travel_dist * self.n
					
				self.iter += 1		# iterating through the number of travels (aka edges of the expanding square)

				# SEND ROBOT to destination calculated previously
				rospy.wait_for_service("/kf1/goto")
				if self.dest.within(self.polygon):
					goto_result = self.go_to(self.dest.x, self.dest.y)
					if(goto_result):	# sending service call succeeded 
						rospy.loginfo("Destination: " + str(self.dest))
						#dist_err = math.sqrt( (
						try:
							result = rospy.wait_for_message("/kf1/waypoint_goto_result", Int8)
						except:
							rospy.loginfo("Timeout waiting for robot to go to destination")
					else:	# sending service call failed
						rospy.loginfo("Failed to go to destination")
						self.driving = False
				else:
					rospy.loginfo("Destination is outside of region")
					self.driving = False

			except rospy.ServiceException, e:
				rospy.loginfo("Goto service call failed")


	# shutdown process
	def shutdown(self):
		rospy.loginfo("Proceeding to shutdown.")
		self.cmd_drive.publish(0, 0)
		rospy.loginfo("Shutdown.")



if __name__ == "__main__":
	rospy.init_node('expanding_square', anonymous=True)

	e_s = expanding_square()
	rospy.on_shutdown(e_s.shutdown)

	while not rospy.is_shutdown() and e_s.driving:
		e_s.search()

	rospy.loginfo("Completed expanding square sweep method.")
