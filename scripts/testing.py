
import GPy
import numpy as np 
import matplotlib.pyplot as plt


x0_range = np.linspace(-5, 5, 15)
x1_range = np.linspace(-5, 5, 15)
X0, X1 = np.meshgrid(x0_range, x1_range)

Y = -1 * np.sqrt(X0**2 + X1**2)
plt.contourf(X0, X1, Y)
plt.colorbar()
plt.savefig("figures/ground_truth.png")
plt.clf()

def create_2d_model(X, Y):
	assert X.shape[1] == 2	# 2-dimensional input: (x0, x1)
	assert Y.shape == (X.shape[0], 1) # 1d output with same
									  # number of obs as X
	model = GPy.models.GPRegression(X, Y, 
		GPy.kern.RBF(2))
	train_result = model.optimize(messages=True,max_f_eval = 1000)
	assert train_result.status == "Converged"
	return model


# # create model using all of the observations:
model_X = np.transpose(
	np.vstack([
		X0.flatten(), 
		X1.flatten()]))
model_Y = Y.reshape(-1, 1)
m = create_2d_model(model_X, model_Y)

# check the model's predictions:
mean, variance = m.predict(model_X)
plt.contourf(X0, X1, mean.reshape(X0.shape))
plt.colorbar()
plt.savefig("figures/predicted.png")


# try creating a model with just a few points:
badmodel_X = np.matrix("4, 4; 3, 3")
badmodel_Y = np.matrix("5.657; 4.243")
m = create_2d_model(badmodel_X, badmodel_Y)
mean, variance = m.predict(model_X)
plt.contourf(X0, X1, mean.reshape(X0.shape))
plt.colorbar()
plt.savefig("figures/predicted.png")

print("script done")