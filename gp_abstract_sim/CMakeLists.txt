cmake_minimum_required(VERSION 2.8.3)
project(gp_abstract_sim)

## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  rospy
  std_msgs
  genmsg
)

find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARIY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

add_library(set_speed_joint_plugin src/set_speed_joint_plugin.cpp)
target_link_libraries(set_speed_joint_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(my_speed_controller_plugin src/my_speed_controller_plugin.cpp)
target_link_libraries(my_speed_controller_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

# add_library(contact SHARED ContactPlugin.cc)
# target_link_libraries(contact ${GAZEBO_LIBRARIES})

add_library(contact src/ContactPlugin.cc)
target_link_libraries(contact ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  DEPENDS
    roscpp
    gazebo_ros
    rospy
    std_msgs
    genmsg
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(my_publisher src/my_publisher.cpp)
target_link_libraries(my_publisher ${catkin_LIBRARIES})
add_dependencies(my_publisher gp_abstract_sim_generate_messages_cpp)
