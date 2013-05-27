##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(turtle_concert)

##############################################################################
# Catkin
##############################################################################

find_package(catkin REQUIRED)
catkin_package()

##############################################################################
# Installs
##############################################################################

install(
    PROGRAMS 
        scripts/init_turtles.py
        scripts/view_implementation
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY concert
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY apps
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY orchestration
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY yaml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
  
