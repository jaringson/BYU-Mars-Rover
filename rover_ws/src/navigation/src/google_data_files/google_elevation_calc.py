import urllib, json
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from math import *
import time
import scipy.io as io

# google API has the following restrictions: 2500 free requests per day, 512 locations per request, 50 requests per second


# Function that uses google API to give an elevation based on a requested latitude and longitude
def get_google_elevation(lat,lon):
	link = 'https://maps.googleapis.com/maps/api/elevation/json?locations='+str(lat)+', '+str(lon)+'&key=AIzaSyD8H2reT6hwm-Ie5_WHywkqKSWPDRjRMPE'
	f = urllib.urlopen(link)
	myfile = f.read()
	data = eval(myfile)
	elevation = data['results'][0]['elevation']
	return elevation

##################################################
## getting initial values (lat, lon, grid size) ##
##################################################

# initial values are bottom left corner of grid, final values are top right corner ... values are in degrees!
initial_lat = 40.254211
initial_lon = -111.632902
num_of_tiles = 50  # this means we will get a nxn grid of elevation points (n = num_of_tiles). Must be less than or equal to 50 because 50^2 = 2500, the max number of requests per day
final_lat = 40.275972
final_lon = -111.598332
reset_lat = initial_lat # reset value which will be used later

lat_increment = (final_lat - initial_lat)/(num_of_tiles - 1)
lon_increment = (final_lon - initial_lon)/(num_of_tiles - 1)

x_data = [] # longitute
y_data = [] # latitude
z_data = [] # altitude (in meters)

# first for loop is for the longitude (later become the 'x' coordinate)
for i in range(0,num_of_tiles):
	altitude = get_google_elevation(initial_lat, initial_lon)
	x_data.append(initial_lon)
	y_data.append(initial_lat)
	z_data.append(altitude)
	# second for loop is for the latitude (later become the 'y' coordinate)
	for i in range(0,num_of_tiles-1):
		initial_lat = initial_lat + lat_increment
		altitude = get_google_elevation(initial_lat, initial_lon)
		x_data.append(initial_lon)
		y_data.append(initial_lat)
		z_data.append(altitude)
	initial_lon = initial_lon + lon_increment
	initial_lat = reset_lat # reset the y coordinate before the next latitude for loop
	time.sleep(.1) # so we do not go over the 50 requests per second restriction

################################################################ 
## turn latitude, longitude, elevation into x,y,z coordinates ##
################################################################

# function that converts latitude and longitude (degrees) into radians
def convert_deg_to_rad(data):
	radian_vector = []
	for i in range(len(data)):
		radian_vector.append(data[i]*(pi/180))
	return radian_vector

x_data = convert_deg_to_rad(x_data)
y_data = convert_deg_to_rad(y_data)

# turning the previous data (vectors) into matrices with dimensions num_of_tiles X num_of_tiles
X = np.array(x_data)
X = np.reshape(x_data, (-1, num_of_tiles))
Y = np.array(y_data)
Y = np.reshape(y_data, (-1, num_of_tiles))
Z = np.array(z_data)
Z = np.reshape(z_data, (-1, num_of_tiles))

# halversine formula, this will be used to make the conversion from lat/lon to distance ... this distance is in km
def halversine(lon1, lat1, lon2, lat2):
	dlon = lon2 - lon1
	dlat = lat2 - lat1
	a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
	c = 2 * atan2(sqrt(a), sqrt(1-a))
	earth_radius = 6371  # km
	gps_distance = earth_radius * c
	return gps_distance

# creating two new nXn matrices that will be used as the x, y coordinate system (Z is already complete)
def coordinate_grid_matrix(coordinate_axis, grid_size):
	'''
	input for coordinate_axis should be either x_coordniate or y_coordinate AS A STRING (Z is already done)
	grid_size is simply the Z matrix (we will get the row and column size from this matrix)
	'''
	row, col = np.shape(grid_size)
	if coordinate_axis == 'x_coordinate':
		x_distance = np.zeros((row, col)) 
		x_increment = halversine(X[0][0], Y[0][0], X[1][0], Y[0][0])
		for i in range(1, row): # starting on row 2 because row one is the origin, aka zero
			for w in range(0, col):
				x_distance[i][w] = x_increment
			x_increment = x_increment + halversine(X[0][0], Y[0][0], X[1][0], Y[0][0])
		return x_distance
	elif coordinate_axis == 'y_coordniate':
		y_distance = np.zeros((row,col))
		y_increment = halversine(X[0][0], Y[0][0], X[0][0], Y[0][1])
		for i in range(1, col): # starting on column 2 because row one is origin, aka zero
			for w in range(0, row):
				y_distance[w][i] = y_increment
			y_increment = y_increment + halversine(X[0][0], Y[0][0], X[0][0], Y[0][1])
		return y_distance

x_coord = coordinate_grid_matrix('x_coordinate', Z)*1000 # converting km to m
y_coord = coordinate_grid_matrix('y_coordniate', Z)*1000 # converting km to m 

#################################
## Saving data to another file ##
#################################
data = {"x":x_coord, "y":y_coord, "z":Z}
# data = {"x":x_coord, "y":y_coord, "z":Z, }
io.savemat('topo_data_google', data)

###############
## Plot Data ##
###############

def plot_map(X, Y, Z):
	fig = plt.figure()
	ax = fig.add_subplot(111, projection='3d')
	ax.plot_wireframe(X, Y, Z) 

	ax.set_xlabel('X (m)')
	ax.set_ylabel('Y (m)')
	ax.set_zlabel('Elevation (m)')

	return plt.show()

plot_map(x_coord, y_coord, Z)
