##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(chatter_concert)

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
        scripts/view_link_graph
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY concert
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY orchestration
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY services
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
  
