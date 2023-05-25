cmake_minimum_required(VERSION 3.1.3)
project(rigid_arm_ikfast_arm_plugin)

if(NOT "${CMAKE_CXX_STANDARD}")
  set(CMAKE_CXX_STANDARD 14)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  pluginlib
  roscpp
  tf2_kdl
  tf2_eigen
  eigen_conversions
)
find_package(LAPACK REQUIRED)

include_directories(${catkin_INCLUDE_DIRS} include)


# This was added to allow using IKFast with other packages
# Note there is some repetition, but this doesn't seem to cause issues

catkin_package(
  # Other pkgs can use ikfast.h
  INCLUDE_DIRS
    include
  LIBRARIES
    ik_solver_lib
)

add_library(ik_solver_lib src/rigid_arm_arm_ikfast_solver.cpp)
target_link_libraries(ik_solver_lib ${catkin_LIBRARIES})

# And include in installs
install(TARGETS ${IKFAST_LIBRARY_NAME} ik_solver_lib LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})


set(IKFAST_LIBRARY_NAME rigid_arm_arm_moveit_ikfast_plugin)
add_library(${IKFAST_LIBRARY_NAME} src/rigid_arm_arm_ikfast_moveit_plugin.cpp)
target_link_libraries(${IKFAST_LIBRARY_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${LAPACK_LIBRARIES})
# suppress warnings about unused variables in OpenRave's solver code
target_compile_options(${IKFAST_LIBRARY_NAME} PRIVATE -Wno-unused-variable -Wno-unused-parameter)

install(TARGETS
  ${IKFAST_LIBRARY_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(
  FILES
  rigid_arm_arm_moveit_ikfast_plugin_description.xml
  DESTINATION
  ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
