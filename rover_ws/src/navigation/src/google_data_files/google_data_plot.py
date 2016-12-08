import scipy.io as io
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

data_set = io.loadmat('topo_data_google.mat')
# print data_set['z'] 

def plot_map(X, Y, Z):
	fig = plt.figure()
	ax = fig.add_subplot(111, projection='3d')
	ax.plot_wireframe(X, Y, Z) 

	ax.set_xlabel('X (m)')
	ax.set_ylabel('Y (m)')
	ax.set_zlabel('Elevation (m)')

	return plt.show()

plot_map(data_set['x'], data_set['y'], data_set['z'])

print data_set['z']
