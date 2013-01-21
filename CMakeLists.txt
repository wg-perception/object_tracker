# object_tracker CMakeLists.txt
cmake_minimum_required(VERSION 2.8.5)
# -----------------------------------------------
# CATKIN
# -----------------------------------------------
project(object_tracker)
find_package(catkin REQUIRED COMPONENTS rospy genmsg object_recognition_core object_recognition_msgs std_msgs 
actionlib_msgs	shape_msgs geometry_msgs visualization_msgs tf dynamic_reconfigure)

# install targets for python
catkin_python_setup()

# Services
add_service_files(DIRECTORY srv FILES EstimateRotation.srv)
add_message_files(DIRECTORY msg FILES RotationParameters.msg RotatingObjects.msg)
generate_messages(DEPENDENCIES geometry_msgs std_msgs object_recognition_msgs sensor_msgs visualization_msgs)

if (${catkin_VERSION} VERSION_GREATER "0.5.28")
    generate_dynamic_reconfigure_options(cfg/ObjectTracker.cfg) 
    catkin_package()
else()
	#add dynamic reconfigure api
	rosbuild_find_ros_package(dynamic_reconfigure)
	include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
	gencfg()
    catkin_stack()
    catkin_project()
endif()

