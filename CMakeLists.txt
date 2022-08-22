cmake_minimum_required(VERSION 2.8.3)

project(oreo_description)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
	roslaunch
	gazebo_dev
	gazebo_msgs
	tf2
	tf
)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES ${PROJECT_NAME}
	CATKIN_DEPENDS roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(omni_contorl src/omni_contorl.cpp)
target_link_libraries(omni_contorl ${catkin_LIBRARIES} ${Boost_LIBRARIES})

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
