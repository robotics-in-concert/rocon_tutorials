cmake_minimum_required(VERSION 2.8.3)
project(gazebo_concert)

find_package(catkin REQUIRED)

catkin_package()

foreach(dir solutions)
  install(DIRECTORY ${dir}
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
endforeach()
                                                                                                                   
                                                                                                                   

