import numpy as np
from math import sqrt

def find_peaks(model, X, x0_range, x1_range):
    """ model:  a GPRegression model that has already been trained.
        X:      a 2-column matrix containing (lat, long) pairs. These points
                should cover the space of the model pretty well (e.g. they
                were generated with meshgrid()).
        
        """

    # choose some points as initialization locations for our
    # gradient-ascending "hill climbers"
    hill_climber_count = 10
    total_points = X.shape[0]
    sample_indices = np.linspace(0, total_points - 1, 
        num=hill_climber_count).astype(int)
    climbers = X[sample_indices, :]
    
    # walk each climber in the direction of the gradient for awhile:
    for row in range(climbers.shape[0]):

        location = climbers[row, :]
        for _ in range(30):

            gradient, _ = model.predictive_gradients(
                    location.reshape(1, 2))
            location += np.squeeze(gradient)

            # ensure we stay inside the region
            location[0] = max(location[0], x0_range[0])
            location[0] = min(location[0], x0_range[-1])
            location[1] = max(location[1], x1_range[0])
            location[1] = min(location[1], x1_range[-1])

    return climbers

