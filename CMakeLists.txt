cmake_minimum_required(VERSION 3.0.0)
project(sas_robot_driver)

#Add custom (non compiling) targets so launch scripts and python files show up in QT Creator's project view.
file(GLOB_RECURSE EXTRA_FILES */*)
add_custom_target(${PROJECT_NAME}_OTHER_FILES ALL WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} SOURCES ${EXTRA_FILES})

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
add_compile_options(-Werror=return-type)

FIND_PACKAGE(Eigen3 CONFIG REQUIRED)
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    pybind11_catkin
    std_msgs
    sensor_msgs
    sas_common
    sas_conversions
    sas_clock
    )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES sas_robot_driver
    CATKIN_DEPENDS roscpp rospy std_msgs pybind11_catkin sensor_msgs sas_common sas_conversions sas_clock
    # DEPENDS dqrobotics
    )

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    )

add_library(${PROJECT_NAME}
    src/sas_robot_driver.cpp
    src/sas_robot_driver_provider.cpp
    src/sas_robot_driver_interface.cpp
    src/sas_robot_driver_ros.cpp
    )

add_executable(sas_robot_driver_ros_composer_node
    src/sas_robot_driver_ros_composer_node.cpp
    src/sas_robot_driver_ros_composer.cpp)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(sas_robot_driver_ros_composer_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    )

target_link_libraries(sas_robot_driver_ros_composer_node
    dqrobotics
    dqrobotics-interface-vrep
    dqrobotics-interface-json11
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    )


#####################
## PYTHON BINDINGS ##
#####################
pybind_add_module(_sas_robot_driver SHARED src/sas_robot_driver_py.cpp src/sas_robot_driver.cpp src/sas_robot_driver_interface.cpp src/sas_robot_driver_provider.cpp)
target_compile_definitions(_sas_robot_driver PRIVATE IS_SAS_PYTHON_BUILD)
# https://github.com/pybind/pybind11/issues/387
target_link_libraries(_sas_robot_driver PRIVATE ${catkin_LIBRARIES} -ldqrobotics)

    
#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
install(PROGRAMS
   scripts/sas_robot_driver_interface_example.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS sas_robot_driver_ros_composer_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
 )

install(TARGETS _sas_robot_driver
  LIBRARY DESTINATION ${PYTHON_INSTALL_DIR}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
    

catkin_python_setup()
