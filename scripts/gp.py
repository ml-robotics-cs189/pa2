from mpl_toolkits.mplot3d import Axes3D
from IPython.display import display
import matplotlib.pyplot as plt
from scipy.io import loadmat
import numpy as np
import GPy
import json

#from numpy import linalg.linalg.LinAlgError


class GP:

	def __init__(self):

		self.gps_data = None
		self.sensor_data = None

		# initialize kernel and model to be used
		self.kernel = GPy.kern.RBF(2)
		self.model = None

		# ground truth of the chlorophyl data
		ground_truth_file = 'Catalina14072016__2009-01-18-04-28-50.mat'
		self.ground_truth = loadmat(ground_truth_file)

		self.lat_mesh = self.ground_truth['latMesh']
		self.lon_mesh = self.ground_truth['lonMesh']
		self.mean_mat = self.ground_truth['zMean']
		var_mat = self.ground_truth['zVar']

		self.mean = []
		self.var = []

		for w in range(len(self.mean_mat)):
			for q in range(len(self.mean_mat[w])):
				self.mean.append(self.mean_mat[w][q])
				self.var.append(var_mat[w][q])

		self.mean = np.array(self.mean)
		self.var = np.array(self.var)

		# TEST
		# # current coord, north coord, east coord, south coord, west coord
		# self.gaussian_proc(np.array( [ [33.44470, -118.48496], [33.44470, -118.48497],
		# 							[33.44471, -118.48496], [33.44470, -118.48495],
		# 							[33.44469, -118.48496] ] ))


	# set new data from the robot
	def add_data(self, new_gps_data, new_sensor_data):
		self.gps_data = new_gps_data
		self.sensor_data = new_sensor_data

		#print("From GP: ", self.gps_data, self.sensor_data)


	# sets the current gaussian process model
	def gaussian_proc(self, curr_pos):

		cur_grad_area = []

		# learn the GP model and display it
		try:
			self.model = GPy.models.GPRegression(self.gps_data, self.sensor_data, self.kernel)
			self.model.optimize(messages=True,max_f_eval = 1000)
			#self.model.plot()
			display(self.model)

			# predict the model with the meshgrid of the lake
			# returns the mean: M and variance: V
			# M, V = self.model.predict(cur_coord)

			# retreive the gradient values of the current coordinate and its neighbors
			M_jac, V_kac = self.model.predictive_gradients(curr_pos)

			gradient = np.squeeze(M_jac)

			if np.linalg.norm(gradient) == 0:
				gradient = [1, 0.5]
		#display(X, mean, "figures/model_%d" % i,
	     #   visited=points,
	      #  arrow=(curr_pos[0], curr_pos[1], gradient[0], gradient[1]))


	    # update the current location:
			next_pos = curr_pos + gradient

			return next_pos

		except np.linalg.LinAlgError:

			#self.model = GPy.models.GPRegression(self.gps_data, self.sensor_data, self.kernel)
			self.model.optimize(messages=True,max_f_eval = 1000)
			#self.model.plot()
			display(self.model)

			# predict the model with the meshgrid of the lake
			# returns the mean: M and variance: V
			# M, V = self.model.predict(cur_coord)

			# retreive the gradient values of the current coordinate and its neighbors
			M_jac, V_kac = self.model.predictive_gradients(curr_pos)

			gradient = np.squeeze(M_jac)

			if np.linalg.norm(gradient) == 0:
				gradient = [1, 0.5]
		#display(X, mean, "figures/model_%d" % i,
	     #   visited=points,
	      #  arrow=(curr_pos[0], curr_pos[1], gradient[0], gradient[1]))


	    # update the current location:
			next_pos = curr_pos + gradient

			return next_pos



	# calculate mean square error evaluation
	def mse_eval( self ):

		X_grid_test = np.transpose(np.vstack([self.lat_mesh.ravel(), self.lon_mesh.ravel()]))

		# predict the model with the meshgrid of the lake
		# returns the mean: M and variance: V
		M, V = self.model.predict(X_grid_test)

		# fit it to the meshgrid
		M_grid = M.reshape(self.lat_mesh.shape)
		V_grid = V.reshape(self.lon_mesh.shape)

		# draw diagrams of GP
		plt.figure("Mean Grid")
		plt.contourf(self.lat_mesh, self.lon_mesh, M_grid)
		plt.colorbar()
		plt.figure("Variance Grid")
		plt.contourf(self.lat_mesh, self.lon_mesh, V_grid)
		plt.colorbar()

		plt.show()

		# mean square error function
		mse = ((self.mean_mat - M_grid)**2).mean(axis=None)

		print("\n" + "Mean square error: " + str(mse) + "\n")
