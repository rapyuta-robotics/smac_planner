cmake_minimum_required(VERSION 3.10)
project(nav2_smac_planner)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        dynamic_reconfigure
        visualization_msgs
        mbf_costmap_core
        nav_msgs
        nav_msgs
        geometry_msgs
        tf2_ros
        costmap_2d
        base_local_planner
        pluginlib
        angles
        eigen_conversions
)

find_package(ompl REQUIRED)

generate_dynamic_reconfigure_options(
        cfg/SmacPlannerHybrid.cfg
        cfg/Smoother.cfg
)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME} # TODO ${PROJECT_NAME}_2d ${PROJECT_NAME}_lattice
        CATKIN_DEPENDS roscpp dynamic_reconfigure visualization_msgs mbf_costmap_core nav_msgs nav_msgs geometry_msgs tf2_ros costmap_2d base_local_planner pluginlib angles
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${OMPL_INCLUDE_DIRS}
)

set(library_name ${PROJECT_NAME})

# Hybrid plugin
add_library(${library_name} SHARED
        src/smac_planner_hybrid.cpp
        src/a_star.cpp
        src/collision_checker.cpp
        src/smoother.cpp
        src/analytic_expansion.cpp
        src/node_hybrid.cpp
        src/node_lattice.cpp
        src/costmap_downsampler.cpp
        src/node_2d.cpp
        src/node_basic.cpp
)

target_link_libraries(${library_name}
        ${catkin_LIBRARIES}
        ${OMPL_LIBRARIES}
)

# TODO 2D plugin
#add_library(${library_name}_2d SHARED
#        src/smac_planner_2d.cpp
#        src/a_star.cpp
#        src/smoother.cpp
#        src/collision_checker.cpp
#        src/analytic_expansion.cpp
#        src/node_hybrid.cpp
#        src/node_lattice.cpp
#        src/costmap_downsampler.cpp
#        src/node_2d.cpp
#        src/node_basic.cpp
#)
#
#target_link_libraries(${library_name}_2d
#        ${catkin_LIBRARIES}
#        ${OMPL_LIBRARIES}
#)
#
## TODO Lattice plugin
#add_library(${library_name}_lattice SHARED
#        src/smac_planner_lattice.cpp
#        src/a_star.cpp
#        src/smoother.cpp
#        src/collision_checker.cpp
#        src/analytic_expansion.cpp
#        src/node_hybrid.cpp
#        src/node_lattice.cpp
#        src/costmap_downsampler.cpp
#        src/node_2d.cpp
#        src/node_basic.cpp
#)
#
#target_link_libraries(${library_name}_lattice
#        ${catkin_LIBRARIES}
#        ${OMPL_LIBRARIES}
#)

install(TARGETS ${library_name} # ${library_name}_2d ${library_name}_lattice
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# TODO
#install(DIRECTORY lattice_primitives/sample_primitives
#        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)

# TODO
#if(CATKIN_ENABLE_TESTING)
#  find_package(rostest REQUIRED)
#  add_subdirectory(test)
#endif()
