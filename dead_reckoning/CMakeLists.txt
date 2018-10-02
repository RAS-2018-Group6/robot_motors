cmake_minimum_required(VERSION 2.8.3)
project(dead_reckoning)

find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs phidgets)



catkin_package(

)

include_directories(
  #include
  ${catkin_INCLUDE_DIRS}
)


add_executable(dead_reckoning_node src/dead_reckoning.cpp)
target_link_libraries(dead_reckoning_node ${catkin_LIBRARIES} ) 
add_dependencies(dead_reckoning_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

