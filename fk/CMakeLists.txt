cmake_minimum_required(VERSION 3.0.2)
project(fk)

# creates several CMake environment variables that give information
# about the found package, used later in the CMake lists
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  cmake_modules #for adding common external packages(eigen in this case)
)

find_package(Eigen REQUIRED)

# required to specify catkin-specific information to the build system 
# which in turn is used to generate pkg-config and CMake files.
# This function must be called before declaring any targets with 
# add_library() or add_executable()
catkin_package()

# requied for including all header files
include_directories(
include ${catkin_INCLUDE_DIRS}
include ${Eigen_INCLUDE_DIRS})

# add any generated executables in the src folder of this package
add_executable(RRR_kinematics src/RRR_kinematics.cpp)

# libraries to link for the executable
target_link_libraries(RRR_kinematics ${catkin_LIBRARIES})

