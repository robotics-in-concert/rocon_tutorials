##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(rocon_gateway_tutorials)

##############################################################################
# Catkin
##############################################################################

find_package(catkin REQUIRED)
catkin_package()

catkin_python_setup()

##############################################################################
# Installs
##############################################################################

install(
    PROGRAMS 
        scripts/advertise_all.py
        scripts/advertise_tutorials.py
        scripts/connect_localhost.py
        scripts/flip_all.py
        scripts/flip_tutorials.py
        scripts/pull_all.py
        scripts/pull_tutorials.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

