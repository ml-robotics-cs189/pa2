import rosbag
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from IPython.display import display
import matplotlib.pyplot as plt
import GPy
import json

class GP:

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
		
		self.sensor_topic = '/kf1/simulated_sensor/raw'
		print("Reading bagfile \"" + self.bagfile + "\"")
		self.bag = rosbag.Bag(self.bagfile)
		sensor_msg_cnt = self.bag.get_message_count(self.sensor_topic)

		self.gps_data = np.zeros((sensor_msg_cnt, 2))
		self.sensor_data = np.zeros((sensor_msg_cnt, 1))

		self.readbag()
		self.gaussian_proc()


	def readbag(self):



		i = 0 	# iterator

		for topic, msg, t in self.bag.read_messages(topics=self.sensor_topic):
			self.sensor_data[i] = [ msg.data ]
			self.gps_data[i] = [ msg.latitude, msg.longitude ]

			i += 1

		self.bag.close()

		#print("Sensor: " , self.sensor_data)
		print("GPS: ", self.gps_data)


	def gaussian_proc(self):
		fig = plt.figure()
		ax = fig.add_subplot(111, projection='3d')

		ax.scatter(self.gps_data[:,0], self.gps_data[:,1], self.sensor_data)

		plt.show()

		kernel = GPy.kern.RBF(2)
		model = GPy.models.GPRegression(self.gps_data, self.sensor_data, kernel)

		model.optimize(messages=True,max_f_eval = 1000)
		model.plot()
		display(model)

		X0 = np.arange(33.44443, 33.44507, 0.00001)
		X1 = np.arange(-118.48498, -118.48413, 0.00001)

		X0_grid, X1_grid = np.meshgrid(X0, X1)

		X_grid_test = np.transpose(np.vstack([X0_grid.ravel(), X1_grid.ravel()]))

		Y, V = model.predict(X_grid_test)

		Y_grid = Y.reshape(X0_grid.shape)
		V_grid = V.reshape(X0_grid.shape)

		plt.contourf(X0_grid, X1_grid, V_grid)
		plt.colorbar()
		plt.figure(0)
		plt.contourf(X0_grid, X1_grid, Y_grid)
		plt.colorbar()

		plt.show()



if __name__ == "__main__":
	gp = GP()


