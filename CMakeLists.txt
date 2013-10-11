cmake_minimum_required(VERSION 2.8.3)
project(cube_finder)

find_package(catkin REQUIRED COMPONENTS pcl_ros roscpp sensor_msgs tf)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES cube_finder
  CATKIN_DEPENDS pcl_ros roscpp sensor_msgs tf
)

include_directories(include ${catkin_INCLUDE_DIRS})

## Declare a cpp library
# add_library(cube_finder
#   src/${PROJECT_NAME}/cube_finder.cpp
# )

add_executable(cube_finder_node src/main.cpp src/cube_finder.cpp)
target_link_libraries(cube_finder_node ${catkin_LIBRARIES})
if(catkin_EXPORTED_TARGETS)
  add_dependencies(cube_finder_node ${catkin_EXPORTED_TARGETS})
endif()

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS cube_finder cube_finder_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_cube_finder.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
