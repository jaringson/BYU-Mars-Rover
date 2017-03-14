# BYU-Mars-Rover

## requirements

ros-kineticc
Ubuntu 16.04

## Installation

```
git clone https://github.com/BYUMarsRover/BYU-Mars-Rover.git
```

```
// Build the shared workspace first
cd ~/shared_ws
catkin_make
source devel/setup.bash

// Then build onboard_ws
cd ~/onbaord_ws
// make sure the imu package is installed
sudo apt-get install ros-kinetic-razor-imu-9dof
// get all your submodules
git submodule update --init --recursive
// then build
catkin_make
source devel/setup.bash
```

You then follow this same process to build the rover_ws which is the workspace to be ran at the base station.
