from scipy.io import loadmat
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from IPython.display import display
import matplotlib.pyplot as plt
from matplotlib import cm


class Eval:

    def __init__( self, ground_truth_file ):

        # temp
        ground_truth_file = 'Catalina14072016__2009-01-18-04-28-50.mat'

        self.ground_truth = loadmat(ground_truth_file)
        # self.ground_truth = tables.openFile(ground_truth_file)

        lat_mat = self.ground_truth['latMesh']
        lon_mat = self.ground_truth['lonMesh']
        mean_mat = self.ground_truth['zMean']
        var_mat = self.ground_truth['zVar']


        # fig = plt.figure()
        # ax = fig.add_subplot(111, projection='3d')

        self.lat = []
        self.lon = []
        self.mean = []
        self.var = []

        for w in range(len(lat_mat)):
            for q in range(len(lat_mat[w])):
                self.lat.append(lat_mat[w][q])
                self.lon.append(lon_mat[w][q])
                self.mean.append(mean_mat[w][q])
                self.var.append(var_mat[w][q])

        # ax.scatter(lat, lon, mean, c='r', marker='^')
        #
        # plt.show()


    def mse_eval( self, model ):

		X0_grid, X1_grid = np.meshgrid(self.lat, self.lon)

		X_grid_test = np.transpose(np.vstack([X0_grid.ravel(), X1_grid.ravel()]))

		M, V = model.predict(X_grid_test)

        mse = ( ( M - self.mean )**2 ).mean(axis=None)

        print(mse)


if __name__ == "__main__":

    evaluation = Eval('temporary file')
