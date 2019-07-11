cmake_minimum_required(VERSION 2.8.3)
project(gazebo_objects)

add_compile_options(-Wall -Wconversion -Wextra -std=c++14)

find_package(catkin REQUIRED COMPONENTS
		roscpp
		moveit_msgs
		moveit_ros_planning_interface)

find_package(gazebo REQUIRED)

find_package(Boost REQUIRED COMPONENTS system)

include_directories(include
	SYSTEM
	${catkin_INCLUDE_DIRS}
	${GAZEBO_INCLUDE_DIRS}
	${Boost_INCLUDE_DIR})

link_directories(SYSTEM
	${GAZEBO_LIBRARY_DIRS})

catkin_package()

add_library(${PROJECT_NAME}_cylinder_plugin
		src/CylinderPlugin.cpp)

add_dependencies(${PROJECT_NAME}_cylinder_plugin
		${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_cylinder_plugin
		${catkin_LIBRARIES}
		${GAZEBO_LIBRARIES})
