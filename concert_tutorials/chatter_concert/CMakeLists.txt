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
        scripts/remocon_dude
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

install(DIRECTORY interactions DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY concert DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
