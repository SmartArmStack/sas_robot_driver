cmake_minimum_required(VERSION 3.20)
project(sas_robot_driver)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#################################
# pybind11 import block [BEGIN] #
# vvvvvvvvvvvvvvvvvvvvvvvvvvvvv #
set(PYBIND11_FINDPYTHON ON) # Fix CMP0148 Warning. https://github.com/pybind/pybind11/issues/4785
add_subdirectory(pybind11)
# ^^^^^^^^^^^^^^^^^^^^^^^^^^^ #
# pybind11 import block [END] #
###############################

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(sas_common REQUIRED)
find_package(sas_core REQUIRED)
find_package(sas_conversions REQUIRED)

####################################
# CPP Shared Library Block [BEGIN] #
# vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv #
# https://ros2-tutorial.readthedocs.io/en/latest/
# The most common use case is to merge everything you need to export
# into the same shared library called ${PROJECT_NAME}.

add_library(${PROJECT_NAME} SHARED
    src/sas_robot_driver_server.cpp
    src/sas_robot_driver_client.cpp
    src/sas_robot_driver_ros.cpp
    )

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

install(
    DIRECTORY include/
    DESTINATION include
    )

target_link_libraries(${PROJECT_NAME}
    PUBLIC
    -ldqrobotics
    Eigen3::Eigen
    rclcpp::rclcpp
    geometry_msgs::geometry_msgs
    std_msgs::std_msgs
    sensor_msgs::sensor_msgs
    sas_common::sas_common
    sas_core::sas_core
    sas_conversions::sas_conversions
    )

install(
    TARGETS ${PROJECT_NAME}
    EXPORT export_${PROJECT_NAME}
    LIBRARY DESTINATION lib
    #ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
    )

# ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ #
# CPP Shared Library Block [END] #
##################################

##### PYBIND11 LIBRARY #####

ament_python_install_package(${PROJECT_NAME})

pybind11_add_module(_${PROJECT_NAME} SHARED
    src/sas_robot_driver_py.cpp
    src/sas_robot_driver_client.cpp
    src/sas_robot_driver_server.cpp
    )

target_link_libraries(_${PROJECT_NAME} PRIVATE ${PROJECT_NAME} -ldqrobotics)

target_include_directories(_${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

install(TARGETS _${PROJECT_NAME}
  DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
)

##END## PYBIND11 LIBRARY #####

##### CPP BINARY #####

add_executable(sas_robot_driver_ros_composer_node
   src/sas_robot_driver_ros_composer_node.cpp
   src/sas_robot_driver_ros_composer.cpp)

target_link_libraries(sas_robot_driver_ros_composer_node
    dqrobotics
    dqrobotics-interface-json11
    ${PROJECT_NAME}
    Eigen3::Eigen
    )

install(TARGETS
  sas_robot_driver_ros_composer_node
  DESTINATION lib/${PROJECT_NAME})

##END## CPP BINARY #####

##### CPP BINARY #####
add_executable(sas_robot_watchdog_commander_node
    src/sas_robot_watchdog_commander_node.cpp)
target_link_libraries(sas_robot_watchdog_commander_node
    ${PROJECT_NAME}
    Eigen3::Eigen
    )
install(TARGETS
  sas_robot_watchdog_commander_node
  DESTINATION lib/${PROJECT_NAME})

##END## CPP BINARY #####

##### CPP BINARY #####

add_executable(sas_robot_driver_ros_example
    src/examples/sas_robot_driver_ros_example.cpp)

target_link_libraries(sas_robot_driver_ros_example
    ${PROJECT_NAME}
    Eigen3::Eigen
    )

install(TARGETS
  sas_robot_driver_ros_example
  DESTINATION lib/${PROJECT_NAME})

##END## CPP BINARY #####

########################
# Launch Block [BEGIN] #
# vvvvvvvvvvvvvvvvvvvv #
# According to https://github.com/SmartArmStack/sas_robot_driver/blob/ros2/CMakeLists.txt
install(DIRECTORY
        launch
        DESTINATION share/${PROJECT_NAME}/
)
# ^^^^^^^^^^^^^^^^^^ #
# Launch Block [END] #
######################

#########################
# Scripts Block [BEGIN] #
# vvvvvvvvvvvvvvvvvvvvv #
# https://ros2-tutorial.readthedocs.io/en/latest/
install(DIRECTORY
        scripts/
        FILE_PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
        DESTINATION lib/${PROJECT_NAME}
)
# ^^^^^^^^^^^^^^^^^^^ #
# Scripts Block [END] #
#######################

ament_package()
