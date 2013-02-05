##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(turtle_stroll)

##############################################################################
# Catkin
##############################################################################

find_package(catkin REQUIRED COMPONENTS roscpp std_srvs)
include_directories(SYSTEM include ${catkin_INCLUDE_DIRS})
catkin_package(CATKIN_DEPENDS roscpp std_srvs)

##############################################################################
# Project
##############################################################################

add_executable(turtle_stroll src/turtle_stroll.cpp)
target_link_libraries(turtle_stroll ${catkin_LIBRARIES})

##############################################################################
# Installs
##############################################################################

install(TARGETS turtle_stroll RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
