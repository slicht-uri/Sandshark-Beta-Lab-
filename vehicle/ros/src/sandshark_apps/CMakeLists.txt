cmake_minimum_required(VERSION 2.8.3)
project(sandshark_apps)

find_package(catkin REQUIRED COMPONENTS sandshark_msgs sandshark_common rosbag) #roscpp)

catkin_package(
  #INCLUDE_DIRS include
  CATKIN_DEPENDS sandshark_msgs sandshark_common
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(sandshark_actionselector_node actionselector/actionselector_node.cpp)
add_executable(sandshark_dynamiccontrol_node dynamiccontrol/dynamiccontrol_node.cpp)
add_executable(sandshark_health_node health/health_node.cpp)
add_executable(sandshark_navigation_node navigation/navigation_node.cpp)
add_executable(sandshark_objectivecontrol_node objectivecontrol/objectivecontrol_node.cpp)
add_executable(sandshark_pathplanning_node pathplanning/pathplanning_node.cpp)

add_executable(sandshark_sharklogger_node logger/sharklogger_node.cpp logger/actuallyusefulrecorder.cpp)


target_link_libraries(sandshark_actionselector_node ${catkin_LIBRARIES})
target_link_libraries(sandshark_dynamiccontrol_node ${catkin_LIBRARIES})
target_link_libraries(sandshark_health_node ${catkin_LIBRARIES})
target_link_libraries(sandshark_navigation_node ${catkin_LIBRARIES})
target_link_libraries(sandshark_objectivecontrol_node ${catkin_LIBRARIES})
target_link_libraries(sandshark_pathplanning_node ${catkin_LIBRARIES})

target_link_libraries(sandshark_sharklogger_node ${catkin_LIBRARIES})


add_dependencies(sandshark_actionselector_node ${catkin_EXPORTED_TARGETS})
add_dependencies(sandshark_dynamiccontrol_node ${catkin_EXPORTED_TARGETS})
add_dependencies(sandshark_health_node ${catkin_EXPORTED_TARGETS})
add_dependencies(sandshark_navigation_node ${catkin_EXPORTED_TARGETS})
add_dependencies(sandshark_objectivecontrol_node ${catkin_EXPORTED_TARGETS})
add_dependencies(sandshark_pathplanning_node ${catkin_EXPORTED_TARGETS})

add_dependencies(sandshark_sharklogger_node ${catkin_EXPORTED_TARGETS})


## Mark executables and/or libraries for installation
install(TARGETS sandshark_actionselector_node sandshark_dynamiccontrol_node sandshark_health_node sandshark_navigation_node sandshark_objectivecontrol_node sandshark_pathplanning_node sandshark_sharklogger_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
   package.xml
   DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/sandshark_apps
)

install(FILES
   config/dynamiccontrol.yaml
   config/motion.yaml
   config/health.yaml
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)