# PartsBasedDetector CMakeLists.txt
cmake_minimum_required(VERSION 2.8.5)
# -----------------------------------------------
# CATKIN
# -----------------------------------------------
project(object_tracker)
find_package(catkin REQUIRED)

if (${catkin_VERSION} VERSION_GREATER "0.5.28")
    catkin_package(INCLUDE_DIRS include)
else()
    catkin_stack()
    catkin_project(${PROJECT_NAME}
                    INCLUDE_DIRS include
    )
endif()

# install targets for all things python
#catkin_python_setup()

# -----------------------------------------------
# COMPILER FLAGS
# -----------------------------------------------

# include all warning types
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Werror -Wno-unused-parameter -pedantic")

# set the build type
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING
        "Choose the type of build. Options are: None, Debug, Release, RelWithDebInfo, MinSizeRel"
    FORCE)
endif()

# add OpenMP support
#if (WITH_OPENMP)
#    find_package(OpenMP)
#    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
#    set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")
#endif()

# add vectorization support
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse4.1")
set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   -msse4.1")

# use highest level of optimization in Release mode
if ("${CMAKE_BUILD_TYPE}" STREQUAL "Release")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
    set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   -O3")
endif ()

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
#add_definitions( -DEIGEN_CORE_H )

# include the dependencies
include_directories(SYSTEM  ${OPENCV_INCLUDE_DIRS}
                            ${Boost_INCLUDE_DIRS}
)
include_directories(include)
#include_directories(${PCL_INCLUDE_DIRS})

# -----------------------------------------------
# SUBDIRECTORIES
# -----------------------------------------------
# add the main executable/library
add_subdirectory(src)
#add_subdirectory(python)
