cmake_minimum_required(VERSION 2.8.3)
project(sandshark_common)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sandshark_msgs
  topic_tools
)

catkin_package(
  INCLUDE_DIRS include mavlink/v1.0/common mavlink/v1.0/adapt eigen_3.2/Eigen linux
  LIBRARIES sandshark_common
  CATKIN_DEPENDS sandshark_msgs
)

include_directories(include mavlink/v1.0/common mavlink/v1.0/adapt eigen_3.2/Eigen linux ${catkin_INCLUDE_DIRS})

add_library(sandshark_common
  condition.cpp
  task_base.cpp
  driver_base.cpp
  main.cpp
  PIDController.cpp
)

IF(IN_SIM)
target_link_libraries(sandshark_common
  #gnustl_shared
  sqlite3
  /usr/lib/libproj.a
  ${catkin_LIBRARIES}
)
ELSE()
target_link_libraries(sandshark_common
  gnustl_shared
  ${CMAKE_SOURCE_DIR}/sandshark_common/libproj.a
  android
  sqlite
  ${catkin_LIBRARIES}
)
ENDIF()

add_dependencies(sandshark_common ${catkin_EXPORTED_TARGETS})

install(TARGETS sandshark_common
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES
   catkin_all.launch
   runtimeEnv.bash
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(FILES
   runtimeEnv.bash
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/misc
)