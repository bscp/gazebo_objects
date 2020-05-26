cmake_minimum_required(VERSION 2.8.3)
project(gazebo_object)

add_compile_options(-Wall -Wconversion -Wextra -std=c++14)

find_package(catkin REQUIRED COMPONENTS
		roscpp
		rospy  # TODO : can this be removed once all is working?
		message_generation
		geometry_msgs
		std_msgs)

find_package(gazebo REQUIRED)

find_package(Boost REQUIRED COMPONENTS system)

add_message_files(
  FILES
  ObjectStateMsg.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

include_directories(include
	SYSTEM
	${catkin_INCLUDE_DIRS}
	${GAZEBO_INCLUDE_DIRS}
	${Boost_INCLUDE_DIR})

link_directories(SYSTEM
	${GAZEBO_LIBRARY_DIRS})

catkin_package(
		CATKIN_DEPENDS message_runtime)

add_library(${PROJECT_NAME}_gazebo_plugin
		src/GazeboPlugin.cpp)

add_dependencies(${PROJECT_NAME}_gazebo_plugin
		${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_gazebo_plugin
		${catkin_LIBRARIES}
		${GAZEBO_LIBRARIES})
