import rosbag
import numpy as np
import matplotlib.pyplot as plt
from IPython.display import display
from scipy.io import loadmat
import GPy
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

def parseBag(bag, file_num):
	gps_data, sensor_data = readBag(bag)
	plotBag(gps_data, sensor_data, True, file_num)


def readBag(bag):

	# water quality data
	gps_data = np.zeros((sensor_msg_cnt, 2))
	sensor_data = np.zeros((sensor_msg_cnt, 1))

	# read the bagfile
	i = 0 	# iterator
	for topic, msg, t in bag.read_messages(topics=sensor_topic):
		sensor_data[i] = [ msg.data ]
		gps_data[i] = [ msg.latitude, msg.longitude ]

		i += 1

	print("finished reading bag file")

	bag.close()

	return gps_data, sensor_data



def plotBag(gps_data, sensor_data, save_to_file=False, file_num = 0):
	# initialize kernel and model to be used
	kernel = GPy.kern.RBF(2)

	# ground truth of the chlorophyl data
	ground_truth_file = 'Catalina14072016__2009-01-18-04-28-50.mat'
	ground_truth = loadmat(ground_truth_file)

	lat_mesh = ground_truth['latMesh']
	lon_mesh = ground_truth['lonMesh']

	# learn the GP model and display it
	model = GPy.models.GPRegression(gps_data, sensor_data, kernel)
	model.optimize(messages=True,max_f_eval = 1000)
	#self.model.plot()
	display(model)

	X_grid_test = np.transpose(np.vstack([lat_mesh.ravel(), lon_mesh.ravel()]))

	# predict the model with the meshgrid of the lake
	# returns the mean: M and variance: V
	M, V = model.predict(X_grid_test)

	# fit it to the meshgrid
	M_grid = M.reshape(lat_mesh.shape)
	V_grid = V.reshape(lon_mesh.shape)

	# draw diagrams of GP
	plt.figure("Mean Grid")
	plt.contourf(lat_mesh, lon_mesh, M_grid)
	plt.colorbar()

	# TEST: shows raw data that was collected
	# fig = plt.figure()
	# ax = fig.add_subplot(111, projection='3d')
	# ax.scatter(gps_data[:,0], gps_data[:,1], sensor_data)

	if save_to_file:
		plt.savefig("sensor_data_" + str(file_num) + ".pdf")
	else:
		plt.show()


if __name__ == "__main__":
	file_num = 0
	for file in bagfiles:
		bag = rosbag.Bag(file)

		sensor_msg_cnt = bag.get_message_count(sensor_topic)

		parseBag(bag, file_num)
		file_num += 1
