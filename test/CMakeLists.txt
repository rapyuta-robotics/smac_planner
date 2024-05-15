# Test utils
catkin_add_gtest(test_utils
  test_utils.cpp
)
target_link_libraries(test_utils
  ${PROJECT_NAME}
)

## Test costmap downsampler
#catkin_add_gtest(test_costmap_downsampler
#  test_costmap_downsampler.cpp
#)
#target_link_libraries(test_costmap_downsampler
#  ${PROJECT_NAME}
#)
#
## Test Node2D
#catkin_add_gtest(test_node2d
#  test_node2d.cpp
#)
#target_link_libraries(test_node2d
#  ${PROJECT_NAME}
#)
#
## Test NodeHybrid
#catkin_add_gtest(test_nodehybrid
#  test_nodehybrid.cpp
#)
#target_link_libraries(test_nodehybrid
#  ${PROJECT_NAME}
#)
#
## Test NodeBasic
#catkin_add_gtest(test_nodebasic
#  test_nodebasic.cpp
#)
#target_link_libraries(test_nodebasic
#  ${PROJECT_NAME}
#)
#
## Test collision checker
catkin_add_gtest(test_collision_checker
  test_collision_checker.cpp
)
target_link_libraries(test_collision_checker
  ${PROJECT_NAME}
)

# Test A*
catkin_add_gtest(test_a_star
  test_a_star.cpp
)
target_link_libraries(test_a_star
  ${PROJECT_NAME}
)
#
## Test SMAC Hybrid
#catkin_add_gtest(test_smac_hybrid
#  test_smac_hybrid.cpp
#)
#target_link_libraries(test_smac_hybrid
#  ${PROJECT_NAME}
#)
#
## Test SMAC 2D
##catkin_add_gtest(test_smac_2d
##  test_smac_2d.cpp
##)
##target_link_libraries(test_smac_2d
##  ${PROJECT_NAME}_2d
##)
#
## Test SMAC lattice
##catkin_add_gtest(test_smac_lattice
##  test_smac_lattice.cpp
##)
##target_link_libraries(test_smac_lattice
##  ${PROJECT_NAME}_lattice
##)
#
## Test SMAC Smoother
#catkin_add_gtest(test_smoother
#  test_smoother.cpp
#)
#target_link_libraries(test_smoother
#  ${PROJECT_NAME}_lattice
#  ${PROJECT_NAME}
#  ${PROJECT_NAME}_2d
#)
#
## Test Lattice node
#catkin_add_gtest(test_lattice_node
#  test_nodelattice.cpp
#)
#target_link_libraries(test_lattice_node
#  ${PROJECT_NAME}
#)
