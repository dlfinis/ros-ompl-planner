cmake_minimum_required(VERSION 2.8.3)
project(ompl_global_planner_base)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  COMPONENTS
     angles
    costmap_2d
    geometry_msgs
    nav_core
    navfn
    nav_msgs
    pluginlib
    roscpp
    tf
    base_local_planner
    rospy
    move_base_msgs
    std_msgs
)


find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(Eigen3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
endif()

include_directories(
  include
  ${Eigen3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

generate_dynamic_reconfigure_options(cfg/Planner.cfg)

find_package(OMPL REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ompl_global_planner_base_lib
  CATKIN_DEPENDS
    costmap_2d
    geometry_msgs
    nav_core
    navfn
    nav_msgs
    pluginlib
    roscpp
    tf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(ompl_global_planner_base_lib
  src/ompl_global_planner_base.cpp
)
target_link_libraries(ompl_global_planner_base_lib ${catkin_LIBRARIES} ${OMPL_LIBRARIES})

add_dependencies(ompl_global_planner_base_lib ${PROJECT_NAME}_gencfg)

install(TARGETS ompl_global_planner_base_lib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(FILES bgp_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
