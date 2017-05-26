import serial
d = serial.Serial('/dev/ttyUSB6', 115200)
d.flush()


#while rospy.not_shutdown():

while True:
    reading = d.readline()
    print reading 

