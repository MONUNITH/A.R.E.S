cmake_minimum_required(VERSION 2.8.3)
project(lidarlite_scan)

# Find catkin dependencies
find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs)

# Call catkin_package
catkin_package()

# Add catkin_INCLUDE_DIRS to the include path
include_directories(${catkin_INCLUDE_DIRS})

set(SOURCE_FILES
    src/main.cpp
    src/lidarLite.h
    src/lidarLite.c
)

# Build the laser_scan_publisher executable
add_executable(laser_scan ${SOURCE_FILES})
# Add a build order dependency on sensor_msgs
# This ensures that sensor_msgs' msg headers are built before your executable
if(sensor_msgs_EXPORTED_TARGETS)
  add_dependencies(laser_scan ${sensor_msgs_EXPORTED_TARGETS})
endif()
# Link against the catkin_LIBRARIES
target_link_libraries(laser_scan wiringPi wiringPiDev pthread ${catkin_LIBRARIES})

# Mark executables and/or libraries for installation
install(TARGETS laser_scan
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
