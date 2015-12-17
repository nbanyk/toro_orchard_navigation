cmake_minimum_required(VERSION 2.8.3)
project(toro)

#make sure ros knows where relevant ros stuff is
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roscpp_serialization
  rospy
  serial
  serial_utils
  std_msgs
  nav_msgs
  sensor_msgs
  tf
  message_generation
  nmea_msgs
  laser_geometry
  pcl_ros
  pcl_conversions
  random_numbers

)
#same as above
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(Eigen REQUIRED)
find_package(PkgConfig) # for bfl (bayesian filtering library)

pkg_check_modules(BFL REQUIRED orocos-bfl) # for bfl

add_message_files(
  FILES
    Odom.msg
    State.msg
    Sweep.msg
)
generate_messages(
	DEPENDENCIES 
	std_msgs geometry_msgs nav_msgs 
)

catkin_package(
  DEPENDS system_lib PCL eigen
  INCLUDE_DIRS include ${PCL_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS}
  LIBRARIES ax2550
  CATKIN_DEPENDS message_runtime geometry_msgs nav_msgs roscpp serial serial_utils tf nmea_msgs pcl_ros bfl
)

#bfl
message("BFL include dirs:" ${BFL_INCLUDE_DIRS})
message("BFL library dirs:" ${BFL_LIBRARY_DIRS})

#so that you don't have to type include/toro every time you include something
include_directories(include $catkin_INCLUDE_DIRS /home/randy/catkin_ws2/src/toro/include/toro ${PCL_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS} /usr/include/pcl-1.7/ /usr/include/eigen3/ ${BFL_INCLUDE_DIRS}/bfl)
link_directories(${PCL_LIBRARY_DIRS} ${BFL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#every time you want to create a new node within this project you have to add these two lines here to get it to build:
#add_executable(<name you'll use to run your node later> src/<cpp file> <path of any files your executable depends on>)
#target_link_libraries(<same name you chose above; called name of executable> ${catkin_LIBRARIES})

#LMS1xx pub node
#add_executable(LMS1xx_pub_node src/LMS1xx_pub_node.cpp src/LMS1xx.cpp include/toro/motorInterface.cpp include/toro/motorInterface.h include/toro/debugOutputObject.cpp include/toro/smSerial.cpp include/toro/smSerialSocketFileInterface.cpp include/toro/tcpIpSocket.cpp include/toro/timer.cpp)
#target_link_libraries(LMS1xx_pub_node ${catkin_LIBRARIES})

#LMS1xx sub node
#add_executable(LMS1xx_sub_node src/LMS1xx_sub_node.cpp src/LMS1xx.cpp include/toro/motorInterface.cpp include/toro/motorInterface.h include/toro/debugOutputObject.cpp include/toro/smSerial.cpp include/toro/smSerialSocketFileInterface.cpp include/toro/tcpIpSocket.cpp include/toro/timer.cpp)
#target_link_libraries(LMS1xx_sub_node ${catkin_LIBRARIES})

#smart motor sub node
add_executable(smart_motor_sub_node src/smart_motor_sub_node.cpp include/toro/motorInterface.cpp include/toro/motorInterface.h include/toro/debugOutputObject.cpp include/toro/smSerial.cpp include/toro/smSerialSocketFileInterface.cpp include/toro/tcpIpSocket.cpp include/toro/timer.cpp)
target_link_libraries(smart_motor_sub_node ${catkin_LIBRARIES})

#smart motor pub node
add_executable(smart_motor_pub_node src/smart_motor_pub_node.cpp include/toro/motorInterface.cpp include/toro/motorInterface.h include/toro/debugOutputObject.cpp include/toro/smSerial.cpp include/toro/smSerialSocketFileInterface.cpp include/toro/tcpIpSocket.cpp include/toro/timer.cpp)
target_link_libraries(smart_motor_pub_node ${catkin_LIBRARIES})

#engine node
#add_executable(engine_node src/engine_node.cpp include/toro/workmanComm.cpp include/toro/workmanParams.cpp include/toro/common/debugOutputObject.cpp include/toro/common/tcpIpSocket.cpp include/toro/common/timer.cpp)
#target_link_libraries(engine_node ${catkin_LIBRARIES})

#driver sub node
add_executable(driver_sub_node src/driver_sub_node.cpp include/toro/workmanComm.cpp include/toro/workmanParams.cpp include/toro/common/debugOutputObject.cpp include/toro/common/tcpIpSocket.cpp include/toro/common/timer.cpp)
target_link_libraries(driver_sub_node ${catkin_LIBRARIES})

#driver pub node
add_executable(driver_pub_node src/driver_pub_node.cpp include/toro/workmanComm.cpp include/toro/workmanParams.cpp include/toro/common/debugOutputObject.cpp include/toro/common/tcpIpSocket.cpp include/toro/common/timer.cpp)
target_link_libraries(driver_pub_node ${catkin_LIBRARIES})

#state pub node
add_executable(state_pub_node src/state_pub_node.cpp include/toro/workmanComm.cpp include/toro/workmanParams.cpp include/toro/common/debugOutputObject.cpp include/toro/common/tcpIpSocket.cpp include/toro/common/timer.cpp)
target_link_libraries(state_pub_node ${catkin_LIBRARIES})

#state sub node
add_executable(state_sub_node src/state_sub_node.cpp include/toro/common/timer.cpp)
target_link_libraries(state_sub_node ${catkin_LIBRARIES})

#toro control node
#add_executable(toro_control_node src/toro_control_node.cpp)
#target_link_libraries(toro_control_node ${catkin_LIBRARIES})

#ebs 289 project node
add_executable(ebs289_project_control_node src/ebs289_project_control_node.cpp)
target_link_libraries(ebs289_project_control_node ${catkin_LIBRARIES})

#gps sub node
add_executable(gps_sub_node src/gps_sub_node.cpp)
target_link_libraries(gps_sub_node ${catkin_LIBRARIES})

#Laser to Point Cloud node
add_executable(LaserToPCL src/LaserToPCL.cpp)
target_link_libraries(LaserToPCL ${catkin_LIBRARIES})

#new SICK laser publisher node
#took out the src/LMS1xx.cpp #add_executable(sweepLaser_pub_node src/sweepLaser_pub_node.cpp src/LMS1xx.cpp include/toro/workmanParams.cpp include/toro/motorInterface.cpp include/toro/motorInterface.h include/toro/debugOutputObject.cpp include/toro/smSerial.cpp include/toro/smSerialSocketFileInterface.cpp include/toro/tcpIpSocket.cpp include/toro/timer.cpp include/toro/sickLMS115Interface.cpp)
add_executable(sweepLaser_pub_node src/sweepLaser_pub_node.cpp include/toro/workmanParams.cpp include/toro/motorInterface.cpp include/toro/motorInterface.h include/toro/debugOutputObject.cpp include/toro/smSerial.cpp include/toro/smSerialSocketFileInterface.cpp include/toro/tcpIpSocket.cpp include/toro/timer.cpp include/toro/sickLMS115Interface.cpp)
target_link_libraries(sweepLaser_pub_node ${catkin_LIBRARIES})

#converts assembled scans into a point cloud node
add_executable(periodic_snapshotter src/periodic_snapshotter.cpp)
target_link_libraries(periodic_snapshotter ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(periodic_snapshotter ${PROJECT_NAME}_gencpp)

#Tests using message filter on a published point cloud so that timing of transform is coordinated
add_executable(messageFilterTest src/messageFilterTest.cpp)
target_link_libraries(messageFilterTest ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(messageFilterTest ${PROJECT_NAME}_gencpp)

#point cloud sub node
add_executable(pointCloud_sub_node src/pointCloud_sub_node.cpp)
target_link_libraries(pointCloud_sub_node ${catkin_LIBRARIES})

#node that converts from point cloud to point cloud2
add_executable(pointCloud_ROSconversion src/pointCloud_ROSconversion.cpp)
target_link_libraries(pointCloud_ROSconversion ${catkin_LIBRARIES} ${PCL_LIBRARIES})

#node that takes pcl cloud points and performs the iterative closest point method on it
add_executable(icp_node src/icp_node.cpp)
target_link_libraries(icp_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})


add_executable(ransac_linefitting_node src/ransac_linefitting_node.cpp)
target_link_libraries(ransac_linefitting_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(ransac_linefitting_node ${PROJECT_NAME}_gencpp)


add_executable(ekf_node src/ekf_node.cpp)
target_link_libraries(ekf_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(ekf_node ${PROJECT_NAME}_gencpp)


add_executable(lateral_line_offset_node src/lateral_line_offset_node.cpp)
target_link_libraries(lateral_line_offset_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(lateral_line_offset_node ${PROJECT_NAME}_gencpp)


add_executable(nearest_tree_pf src/nearest_tree_pf.cpp)
target_link_libraries(nearest_tree_pf ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(nearest_tree_pf ${PROJECT_NAME}_gencpp)

add_executable(navigation_controller_node src/navigation_controller_node.cpp include/toro/workmanParams.cpp)
target_link_libraries(navigation_controller_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(navigation_controller_node ${PROJECT_NAME}_gencpp)

add_executable(data_collector1 src/data_collector1.cpp)
target_link_libraries(data_collector1 ${catkin_LIBRARIES})

add_executable(data_collector2 src/data_collector2.cpp)
target_link_libraries(data_collector2 ${catkin_LIBRARIES})

add_executable(data_collector3 src/data_collector3.cpp)
target_link_libraries(data_collector3 ${catkin_LIBRARIES})

add_executable(data_collector4 src/data_collector4.cpp)
target_link_libraries(data_collector4 ${catkin_LIBRARIES})

add_executable(data_collector5 src/data_collector5.cpp)
target_link_libraries(data_collector5 ${catkin_LIBRARIES})

add_executable(data_collector6 src/data_collector6.cpp)
target_link_libraries(data_collector6 ${catkin_LIBRARIES})

add_executable(data_collector7 src/data_collector7.cpp)
target_link_libraries(data_collector7 ${catkin_LIBRARIES})

add_executable(data_collector8 src/data_collector8.cpp)
target_link_libraries(data_collector8 ${catkin_LIBRARIES})
