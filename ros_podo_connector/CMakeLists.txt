cmake_minimum_required(VERSION 2.8.3)
project(ros_podo_connector)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  tf
  nav_msgs
)

generate_messages(DEPENDENCIES std_msgs)




include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

add_executable(ros_podo_connector src/ros_podo_connector.cpp)
add_dependencies(ros_podo_connector drc_podo_connector_generate_messages_cpp)
target_link_libraries(ros_podo_connector ${catkin_LIBRARIES})

add_executable(exportCSV src/exportCSV.cpp)
add_dependencies(exportCSV drc_podo_connector_generate_messages_cpp)
target_link_libraries(exportCSV ${catkin_LIBRARIES})


catkin_package(
  DEPENDS
    roscpp
    std_msgs
)
