cmake_minimum_required(VERSION 3.8)
project(sas_robot_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(pybind11 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(sas_common REQUIRED)
find_package(sas_core REQUIRED)
find_package(sas_conversions REQUIRED)
find_package(Eigen3 REQUIRED)

##### CPP LIBRARY #####

add_library(${PROJECT_NAME} SHARED
    src/sas_robot_driver_server.cpp
    src/sas_robot_driver_client.cpp
    src/sas_robot_driver_ros.cpp
    )

ament_target_dependencies(${PROJECT_NAME} rclcpp geometry_msgs std_msgs sensor_msgs sas_common sas_core sas_conversions)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 rclcpp geometry_msgs std_msgs sensor_msgs sas_common sas_core sas_conversions)

install(
    DIRECTORY include/
    DESTINATION include
    )

target_link_libraries(${PROJECT_NAME}
    -ldqrobotics
    Eigen3::Eigen
    )

install(
    TARGETS ${PROJECT_NAME}
    EXPORT export_${PROJECT_NAME}
    LIBRARY DESTINATION lib
    #ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
    )

##END## CPP LIBRARY #####

##### PYBIND11 LIBRARY #####

ament_python_install_package(${PROJECT_NAME})

pybind11_add_module(_${PROJECT_NAME} SHARED src/sas_robot_driver_py.cpp src/sas_robot_driver_client.cpp src/sas_robot_driver_server.cpp)

target_include_directories(_${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

target_compile_definitions(_${PROJECT_NAME} PRIVATE IS_SAS_PYTHON_BUILD)
# https://github.com/pybind/pybind11/issues/387
target_link_libraries(_${PROJECT_NAME} PRIVATE ${PROJECT_NAME} -ldqrobotics)

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
    dqrobotics-interface-vrep
    dqrobotics-interface-json11
    ${PROJECT_NAME}
    Eigen3::Eigen
    )

install(TARGETS
  sas_robot_driver_ros_composer_node
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

##### LAUNCH FILES #####

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

##END## LAUNCH FILES #####

ament_package()
