from mpl_toolkits.mplot3d import Axes3D
from IPython.display import display
import matplotlib.pyplot as plt
import GPy
import json

class GP:

	def __init__(self, gps_data, sensor_data):
		self.gps_data = gps_data
		self.sensor_data = sensor_data

	def gaussian_proc(self, save_to_file=False):

		print("here")

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

		print("finished")

		return model
