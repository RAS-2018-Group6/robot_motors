cmake_minimum_required(VERSION 2.8.3)
project(test_controller)

find_package(catkin REQUIRED COMPONENTS 
	roscpp 
	geometry_msgs

)



catkin_package(
  INCLUDE_DIRS include
  DEPENDS
  CATKIN_DEPENDS roscpp geometry_msgs
)





include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)



add_executable(circle_test_controller src/circle_controller.cpp)
target_link_libraries(circle_test_controller ${catkin_LIBRARIES} ) 
add_dependencies(circle_test_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

