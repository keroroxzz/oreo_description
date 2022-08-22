# OREO Gazebo model V2.3.0

Last update: 2022/07/17

# Install
## 1. Install ros-control and controllers packages.

	sudo apt-get install ros-noetic-ros-control ros-noetic-ros-controllers
	cd ~/catkin_ws/src
		
## 2. Install the following dependencise:
	
### realsense_gazebo_plugin
		
	git clone https://github.com/pal-robotics/realsense_gazebo_plugin
			
### velodyne_simulator
		
	git clone https://bitbucket.org/DataspeedInc/velodyne_simulator.git
			
### kinova-ros
		
	git clone https://github.com/Kinovarobotics/kinova-ros.git	


## 3. Clone package into src, and catkin_make.
		
	cd ~/catkin_ws/src
	git clone https://github.com/keroroxzz/oreo_description.git	
	cd ~/catkin_ws
	catkin_make

# Use
## 1. Launch the default OREO.
		
	roslaunch oreo_description oreo.launch

## 2. Launch the OREO with kinova arm.
		
	roslaunch oreo_description oreo.launch kinova:=True
	
