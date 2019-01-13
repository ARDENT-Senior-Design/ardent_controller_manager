cmake_minimum_required(VERSION 2.8.3)
project(ardent_controller_manager)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS cmake_modules ardent ardent_controllers ardent_component_msgs sensor_msgs realtime_tools roscpp pluginlib rostest)

find_package(Eigen3 REQUIRED)
find_package(TinyXML REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread chrono date_time atomic)

catkin_package(
    DEPENDS TinyXML
    CATKIN_DEPENDS ardent ardent_controllers ardent_component_msgs sensor_msgs realtime_tools roscpp pluginlib
    INCLUDE_DIRS include
    LIBRARIES pr2_controller_manager
)

add_library(ardent_controller_manager
    src/controller_manager.cpp
    src/scheduler.cpp)

target_link_libraries(ardent_controller_manager ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${TinyXML_LIBRARIES})

install(TARGETS ardent_controller_manager 
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})



install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
