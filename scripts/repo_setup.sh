sudo apt-get install ros-kinetic-gpsd-client
sudo apt-get install ros-kinetic-razor-imu-9dof
sudo apt-get install ros-kinetic-trac-ik-lib
cd ~/BYU-Mars-Rover/shared_ws/
catkin_make
source devel/setup.bash
cd ../onboard_ws/
catkin_make
source devel/setup.bash
cd ../rover_ws/
catkin_make

echo "source ~/BYU-Mars-Rover/shared_ws/devel/setup.bash" >> ~/.bashrc
echo "source ~/BYU-Mars-Rover/onboard_ws/devel/setup.bash" >> ~/.bashrc
echo "source ~/BYU-Mars-Rover/rover_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
