cmake_minimum_required(VERSION 2.8.3)
project(sandshark_drivers)

find_package(catkin REQUIRED COMPONENTS sandshark_msgs sandshark_common roscpp)

catkin_package(
  CATKIN_DEPENDS sandshark_msgs sandshark_common
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(sandshark_altimeter_node altimeter/altimeter_node.cpp)
add_executable(sandshark_currentsense_node currentsense/currentsense_node.cpp)
add_executable(sandshark_depth_node depth/depth_node.cpp depth/MS89BSD.cpp depth/MS5803.cpp)
add_executable(sandshark_environmental_node environmental/environmental_node.cpp)
add_executable(sandshark_gasgauge_node gasgauge/gasgauge_node.cpp gasgauge/BQ34Z100.cpp)
add_executable(sandshark_gps_node gps/gps_node.cpp)
add_executable(sandshark_led_node led/led_node.cpp led/SX1509.cpp)
add_executable(sandshark_mavlink_node mavlink/mavlink_node.cpp mavlink/ellipsoidFit.cpp)
add_executable(sandshark_tailcone_node tailcone/tailcone_node.cpp)

target_link_libraries(sandshark_altimeter_node ${catkin_LIBRARIES})
target_link_libraries(sandshark_currentsense_node ${catkin_LIBRARIES})
target_link_libraries(sandshark_depth_node ${catkin_LIBRARIES})
target_link_libraries(sandshark_environmental_node ${catkin_LIBRARIES})
target_link_libraries(sandshark_gasgauge_node ${catkin_LIBRARIES})
target_link_libraries(sandshark_gps_node ${catkin_LIBRARIES})
target_link_libraries(sandshark_led_node ${catkin_LIBRARIES})
target_link_libraries(sandshark_mavlink_node ${catkin_LIBRARIES})
target_link_libraries(sandshark_tailcone_node ${catkin_LIBRARIES})

add_dependencies(sandshark_altimeter_node ${catkin_EXPORTED_TARGETS})
add_dependencies(sandshark_currentsense_node ${catkin_EXPORTED_TARGETS})
add_dependencies(sandshark_depth_node MS89BSD MS5803 ${catkin_EXPORTED_TARGETS})
add_dependencies(sandshark_environmental_node ${catkin_EXPORTED_TARGETS})
add_dependencies(sandshark_gasgauge_node ${catkin_EXPORTED_TARGETS})
add_dependencies(sandshark_gps_node ${catkin_EXPORTED_TARGETS})
add_dependencies(sandshark_led_node ${catkin_EXPORTED_TARGETS})
add_dependencies(sandshark_mavlink_node ${catkin_EXPORTED_TARGETS})
add_dependencies(sandshark_tailcone_node ${catkin_EXPORTED_TARGETS})

 install(TARGETS sandshark_altimeter_node sandshark_currentsense_node sandshark_depth_node sandshark_environmental_node sandshark_gasgauge_node sandshark_gps_node sandshark_led_node sandshark_mavlink_node sandshark_tailcone_node
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

#only compile the motion driver when compiling for the adapt module
IF(NOT IN_SIM)
add_executable(sandshark_motion_node motion/motion_node.cpp)
target_link_libraries(sandshark_motion_node ${catkin_LIBRARIES})
add_dependencies(sandshark_motion_node ${catkin_EXPORTED_TARGETS})
install(TARGETS sandshark_motion_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
ELSE()
ENDIF()

install(FILES
   package.xml
   DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/sandshark_drivers
)
