cmake_minimum_required(VERSION 2.8.3)
project(detect_planner)

find_package(catkin REQUIRED
  COMPONENTS
    base_local_planner
    costmap_2d
    nav_core
    pluginlib
    roscpp
    tf2
    tf2_geometry_msgs
    tf2_ros
)

include_directories(
    include 
    ${catkin_INCLUDE_DIRS}
    )
add_definitions(${EIGEN3_DEFINITIONS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES detect_planner
  CATKIN_DEPENDS
    base_local_planner
    costmap_2d
    nav_core
    pluginlib
    roscpp
    tf2
    tf2_ros
)

add_library(detect_planner src/detect_planner.cpp)
add_dependencies(detect_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(detect_planner
    ${catkin_LIBRARIES}
    )

add_executable(detect_planner_node src/detect_planner_node.cpp)
add_dependencies(detect_planner_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(detect_planner_node detect_planner)
set_target_properties(detect_planner_node PROPERTIES OUTPUT_NAME detect_planner)

install(
    TARGETS detect_planner_node
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(TARGETS detect_planner
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
       )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)



