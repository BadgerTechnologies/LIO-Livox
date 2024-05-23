cmake_minimum_required(VERSION 2.8.3)
project(lio_livox)

# SET(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)

#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# Large warnings are printed when compiling:
#   "The practice of declaring the Bind placeholders (_1, _2, ...) in the
#    global namespace is deprecated. Please use <boost/bind/bind.hpp> + using
#    namespace boost::placeholders, or define BOOST_BIND_GLOBAL_PLACEHOLDERS
#    to retain the current behavior."
# These warnings come from ROS (noetic) files that we do not want to modify
# and make it difficult to find other compile errors during development.
add_definitions(-DBOOST_BIND_GLOBAL_PLACEHOLDERS)

find_package(catkin REQUIRED COMPONENTS
  	     message_generation
  	     geometry_msgs
  	     nav_msgs
  	     sensor_msgs
  	     roscpp
  	     rospy
  	     std_msgs
  	     tf
  	     tf_conversions
  	     rosbag
  	     eigen_conversions
  	     pcl_conversions
  	     pcl_ros
  	     message_filters
  	     std_srvs)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(Ceres REQUIRED)
find_package(OpenCV REQUIRED)
find_package(SuiteSparse REQUIRED)

include_directories(include
        	    ${catkin_INCLUDE_DIRS}
		    ${EIGEN3_INCLUDE_DIR}
		    ${PCL_INCLUDE_DIRS}
        	    ${CERES_INCLUDE_DIRS}
		    ${OpenCV_INCLUDE_DIRS}
        	    ${SUITESPARSE_INCLUDE_DIRS})

##################
## ROS messages ##
##################

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs message_runtime 
  	       pcl_conversions pcl_ros message_filters std_srvs tf_conversions tf 
	       eigen_conversions DEPENDS PCL OpenCV INCLUDE_DIRS include)

add_executable (ScanRegistration 
		src/lio/ScanRegistration.cpp
		src/lio/LidarFeatureExtractor.cpp
        	src/segment/segment.cpp
        	src/segment/pointsCorrect.cpp)
target_link_libraries(ScanRegistration ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable (PoseEstimation 
		src/lio/PoseEstimation.cpp
		src/lio/Estimator.cpp
		src/lio/IMUIntegrator.cpp
		src/lio/ceresfunc.cpp 
		src/lio/Map_Manager.cpp)
target_link_libraries(PoseEstimation ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES})

### Install

install(TARGETS
  ScanRegistration
  PoseEstimation
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

