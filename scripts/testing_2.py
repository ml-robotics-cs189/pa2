from math import sqrt
import numpy as np
import matplotlib.pyplot as plt
import GPy
from planning.find_peaks import find_peaks


def create_2d_model(X, Y):
	assert X.shape[1] == 2	# 2-dimensional input: (x0, x1)
	assert Y.shape == (X.shape[0], 1) # 1d output with same
									  # number of obs as X
	model = GPy.models.GPRegression(X, Y, 
		GPy.kern.RBF(2))
	train_result = model.optimize(max_f_eval = 1000)
	assert train_result.status == "Converged"
	return model


def display(X, Y, filename, visited=None, arrow=None, levels=40, save=True):

    # if X is a list containing the two meshgrids:
    if isinstance(X, list):
        c = plt.contourf(X[0], X[1], Y, levels)
    
    # otherwise, X is a matrix with two columns:
    # the X0 and X1 values. Unpack and reshape into squares.
    else:
        X0 = X[:, 0]
        X1 = X[:, 1]
        s = int(sqrt(X0.size))

        c = plt.contourf(
            X0.reshape(s, s), 
            X1.reshape(s, s), 
            Y.reshape(s, s),
            levels)

    plt.colorbar(c)
    plt.clim(-8, 0)

    if visited is not None:
        for i in range(visited.shape[0]):
            x, y = visited[i, :]
            c = plt.Circle((x, y), 0.1, color="black")
            plt.gcf().gca().add_artist(c)

    if arrow is not None:
        plt.arrow(*arrow, width=0.005)

    if save:
        plt.savefig(filename)
        plt.clf()


# generate ground truth
x0_range = np.linspace(-5, 5, 15)
x1_range = np.linspace(-5, 5, 15)
X0, X1 = np.meshgrid(x0_range, x1_range)
Y = -1 * np.sqrt(X0**2 + X1**2)

# reshape into good inputs for the GPy interface:
X = np.transpose(
    np.vstack([
        X0.flatten(), 
        X1.flatten()]))
Y = Y.reshape(-1, 1)

display(X, Y, "figures/ground truth.png")


curr_pos = np.array([3, 3.5])
points = None
y_vals = None
for i in range(20):

    # fetch y-value for current location
    curr_y = -1 * sqrt(curr_pos[0]**2 + curr_pos[1]**2)

    print "Current location: ", curr_pos, "Y-value: ", curr_y

    # add the current location & y-value to our dataset:
    if points is None:
        points = curr_pos.reshape(1, 2)
    else:
        points = np.vstack([points, curr_pos])
    if y_vals is None:
        y_vals = np.matrix(curr_y)
    else:
        y_vals = np.vstack([y_vals, np.matrix(curr_y)])

    # compute a new model given the current data:
    model = create_2d_model(points, y_vals)
    mean, var = model.predict(X)
    gradient, _ = model.predictive_gradients(curr_pos.reshape(1, 2))
    gradient = np.squeeze(gradient)
    if np.linalg.norm(gradient) == 0: 
        gradient = [1, 0.5] # arbitrary hack (to avoid 0 gradient)
    
    peaks = find_peaks(model, X, x0_range, x1_range)
    
    display(X, mean, "blablabla", visited=points, 
        arrow=(curr_pos[0], curr_pos[1], gradient[0], gradient[1]), 
        save=False)
    
    min_dist, min_ind = 99999, -1
    print "peaaks shape", peaks.shape
    for row in range(peaks.shape[0]):
        x = peaks[row, 0]
        y = peaks[row, 1]
        c = plt.Circle((x, y), 0.1, color="gray")
        plt.gcf().gca().add_artist(c)

        dist = np.linalg.norm(curr_pos - peaks[row, :])
        if dist < min_dist:
            min_dist = dist
            min_ind = row

    # plot the nearest peak:
    x = peaks[min_ind, 0]
    y = peaks[min_ind, 1]
    c = plt.Circle((x, y), 0.2, color="darkgray")
    plt.gcf().gca().add_artist(c)

    # plot arrow to the nearest peak:
    direction = peaks[min_ind, :] - curr_pos
    direction = direction * (1 / (np.linalg.norm(direction) * 1.5))

    plt.arrow(curr_pos[0], curr_pos[1], direction[0], direction[1], width=0.005, color="darkgray")

    plt.savefig("figures/model_%d.png" % i)
    plt.clf()


    # update the current location according to the LARGER vector:
    if np.linalg.norm(direction) > np.linalg.norm(gradient):
        curr_pos = curr_pos + direction
    else:
        curr_pos = curr_pos + gradient
    
    # curr_pos = curr_pos + gradient #
    # curr_pos = curr_pos + direction  # use nearest peak instead
    


    