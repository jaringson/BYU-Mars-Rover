
# Install ROS Kinetic on 16.04
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-get update
sudo apt-get install ros-kinetic-desktop-full
sudo rosdep init
rosdep update
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt-get install python-rosinstall

# Install Dependencies
sudo apt-get install ros-kinetic-joy
sudo apt-get install ros-kinetic-urdfdom-py
sudo apt-get install ros-kinetic-trac-ik
sudo apt-get install ros-kinetic-moveit
sudo apt-get install libgps-dev
sudo apt-get install ros-kinetic-tf2-geometry-msgs
sudo apt-get install ros-kinetic-gps-umd
sudo apt-get install ros-kinetic-geographic-msgs
sudo apt-get install ros-kinetic-orocos-kdl
sudo apt-get install ros-kinetic-orocos-kinematics-dynamics

sudo apt-get install arduino arduino.core
