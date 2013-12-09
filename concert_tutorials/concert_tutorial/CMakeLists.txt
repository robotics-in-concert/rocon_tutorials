##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(concert_tutorial)

##############################################################################
# Catkin
##############################################################################

find_package(catkin REQUIRED)
catkin_package()

##############################################################################
# Installs
##############################################################################

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY concert
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY services
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
  
