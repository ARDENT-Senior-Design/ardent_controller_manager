cmake_minimum_required(VERSION 2.8.3)
project(ardent_controller_manager)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS rospy cmake_modules ardent_model ardent_controller_interface ardent_component_msgs sensor_msgs realtime_tools roscpp pluginlib)

find_package(Eigen3 REQUIRED)
find_package(TinyXML REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread chrono date_time atomic)

include_directories(
   include 
   ${Boost_INCLUDE_DIRS} 
   ${catkin_INCLUDE_DIRS} 
   ${EIGEN3_INCLUDE_DIRS})

catkin_package(
    DEPENDS TinyXML
    CATKIN_DEPENDS ardent_model ardent_controller_interface ardent_component_msgs sensor_msgs realtime_tools roscpp pluginlib
    INCLUDE_DIRS include
    LIBRARIES ardent_controller_manager
)

add_library(ardent_controller_manager
    src/controller_manager.cpp
    src/scheduler.cpp)

target_link_libraries(ardent_controller_manager ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${TinyXML_LIBRARIES})

catkin_python_setup()

file(GLOB PYTHON_SCRIPTS RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" 
   "${CMAKE_CURRENT_SOURCE_DIR}/scripts/*")
install(PROGRAMS ${PYTHON_SCRIPTS}
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(FILES controller_manager.launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(TARGETS ardent_controller_manager 
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(FILES controller_manager.launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
