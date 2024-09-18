cmake_minimum_required(VERSION 3.0.2)
project(camera_throttle_nodelet)

find_package(catkin REQUIRED COMPONENTS
   dynamic_reconfigure
   image_transport
   nodelet
   pluginlib
   roscpp
   sensor_msgs 
   )

generate_dynamic_reconfigure_options(cfg/CameraThrottle.cfg)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_library(camera_throttle_nodelet src/camera_throttle.cpp)
add_dependencies(camera_throttle_nodelet ${PROJECT_NAME}_gencfg)
target_link_libraries(camera_throttle_nodelet ${catkin_LIBRARIES})

install(TARGETS camera_throttle_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES nodelet.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
