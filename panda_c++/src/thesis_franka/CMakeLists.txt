cmake_minimum_required(VERSION 2.8.3)
project(thesis_franka)

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  franka_hw
  franka_msgs
  hardware_interface
  roscpp
  rospy
  geometry_msgs
  message_generation
)

find_package(Eigen3 REQUIRED)
find_package(Franka 0.5.0 REQUIRED)

add_service_files(
  FILES
  GripperData.srv
)

generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES thesis_franka
  CATKIN_DEPENDS
    franka_hw
    geometry_msgs
    franka_msgs
    hardware_interface
    roscpp
    message_runtime
  DEPENDS Franka
 )

include_directories(
        # include
        ${catkin_INCLUDE_DIRS}
        /home/palash/thesisROS/panda_c++/devel/include
)

add_executable(cartesian_impedance_control_motionPlanner src/cartesian_impedance_control_motionPlanner.cpp src/examples_common.cpp src/MotionPlanner.cpp)

target_link_libraries(cartesian_impedance_control_motionPlanner PUBLIC
  ${Franka_LIBRARIES}
  ${catkin_LIBRARIES}
)

target_include_directories(cartesian_impedance_control_motionPlanner SYSTEM PUBLIC
  ${Franka_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)