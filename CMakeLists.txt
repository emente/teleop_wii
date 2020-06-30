cmake_minimum_required(VERSION 2.8.3)

project(teleop_wii)

#---first: find_package, catkin_package, include_directories

find_package(catkin REQUIRED COMPONENTS
    cmake_modules
    roscpp
    geometry_msgs
)

catkin_package(
    INCLUDE_DIRS
        /root/otomo_ws/external_src/wiiuse/src
    LIBRARIES
        wiiuse
    CATKIN_DEPENDS
        roscpp
        geometry_msgs
)

include_directories(
    /root/otomo_ws/external_src/wiiuse/src
    ${catkin_INCLUDE_DIRS}
)

#---second: add_executable() add_libraray target_link_libraries

#add_library(${PROJECT_NAME}_core STATIC
#   include/wiiuse.c
#)

add_executable(${PROJECT_NAME}
    src/wii.cpp 
)


#add_dependencies(${PROJECT_NAME}_core
#  ${catkin_EXPORTED_TARGETS}
#)
#add_dependencies(${PROJECT_NAME}_node
#  ${catkin_EXPORTED_TARGETS}
#)


target_link_libraries(${PROJECT_NAME}
    wiiuse
    ${catkin_LIBRARIES}
)

