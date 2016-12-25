import scipy.io as io
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

'''
This is a file that creates the same plot as google_elevation_calc.py ... the benefit of this file is that google_elevation_calc.py has a long run time and can only be ran once a day if you are pulling 2500 data points, so this file allows you to see the graph (and data if you choose) of the data last called from google_elevation_calc.py as many times as you want in a 24 hr period while minimalizing run time
'''

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
