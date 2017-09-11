# extract_rosbag
Reads, processes, and displays data contained within a ROSBag file

## System Requirements

* Ubuntu 16.04 (native or via Docker)

## Software Requirements

* ROS
* Python 2.7

## Required Python packages

Most of these can be installed with a simple 'pip install MODULE' command

* numpy
* cv2
* rosbag
* rospkg
* matplotlib (needs version 2.0+)
* cv_bridge (apt-get install ros-kinetic-cv-bridge)

## To Setup

Perform a `catkin_make` and `source devel/setup.bash`

## Command Line via roslaunch

To extract a rosbag's images to an output folder:

`roslaunch extract_rosbag extract_rosbag.launch rosbag:=<FULL PATH TO BAGFILE> outdir:=<FULL PATH TO OUTPUT FOLDER>`

Note: make sure your output folder already exists.
