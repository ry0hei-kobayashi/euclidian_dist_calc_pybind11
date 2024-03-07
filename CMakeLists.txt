cmake_minimum_required(VERSION 3.12)
project(deformation_detector)

find_package(catkin REQUIRED COMPONENTS
  pybind11_catkin

  roscpp

  pcl_ros
  pcl_conversions
  
  geometry_msgs
  sensor_msgs
  std_msgs
)

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

catkin_python_setup()

catkin_package(
  LIBRARIES deformation_detector
  CATKIN_DEPENDS pybind11_catkin roscpp 
  DEPENDS PCL 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)

#for pybind
pybind_add_module(deformation_detection
  src/deformation.cpp
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

target_link_libraries(deformation_detection
  PRIVATE
  ${catkin_LIBRARIES} 
  ${PCL_LIBRARIES} 
  ${OpenCV_LIBRARIES}
  ${EIGEN_INCLUDE_DIRS}
) 


# Install targets
install(TARGETS deformation_detection
	LIBRARY DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
	
