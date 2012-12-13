# object_tracker CMakeLists.txt
cmake_minimum_required(VERSION 2.8.5)
# -----------------------------------------------
# CATKIN
# -----------------------------------------------
project(object_tracker)
find_package(catkin REQUIRED COMPONENTS rospy genmsg object_recognition_core object_recognition_msgs pcl pcl_ros std_msgs geometry_msgs visualization_msgs tf dynamic_reconfigure)

# install targets for all things python
catkin_python_setup()

# Services
add_service_files(DIRECTORY srv FILES EstimateRotation.srv)
add_message_files(DIRECTORY msg FILES RotationParameters.msg RotatingObjects.msg)
generate_messages(DEPENDENCIES geometry_msgs std_msgs object_recognition_msgs sensor_msgs visualization_msgs)

if (${catkin_VERSION} VERSION_GREATER "0.5.28")
    #generate_dynamic_reconfigure_options(cfg/RotatingObjectTracker.cfg) 
    catkin_package(INCLUDE_DIRS include)
else()
	#add dynamic reconfigure api
	rosbuild_find_ros_package(dynamic_reconfigure)
	include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
	#gencfg()
    catkin_stack()
    catkin_project(${PROJECT_NAME}
                    INCLUDE_DIRS include
    )
endif()

# -----------------------------------------------
# DEPENDENCIES
# -----------------------------------------------
# find the dependencies
find_package(Boost COMPONENTS system filesystem signals thread REQUIRED)
find_package(OpenCV REQUIRED)
#find_package(PCL REQUIRED)
find_package(Eigen REQUIRED)


include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(-DEIGEN_USE_NEW_STDVECTOR
                -DEIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET)

# include the dependencies
include_directories(SYSTEM  ${OPENCV_INCLUDE_DIRS}
                            ${Boost_INCLUDE_DIRS}
)
#include_directories(include
#					${catkin_INCLUDE_DIRS})


# -----------------------------------------------
# SUBDIRECTORIES
# -----------------------------------------------
# add the main executable/library

add_subdirectory(src)

