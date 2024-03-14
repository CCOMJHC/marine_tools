cmake_minimum_required(VERSION 2.8.3)
project(marine_tools)

find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    marine_acoustic_msgs
    nmea_msgs
    pcl_ros
    project11_msgs
    rosbag
    roscpp
    rospy
    tf2
    tf2_ros
    visualization_msgs
)

catkin_python_setup()

catkin_package(
    CATKIN_DEPENDS
        geometry_msgs
        marine_acoustic_msgs
        nmea_msgs
        pcl_ros
        project11_msgs
        roscpp
        tf2
        tf2_ros
        visualization_msgs
    INCLUDE_DIRS 
    #include
)

include_directories(
    #include
    ${catkin_INCLUDE_DIRS}
)


add_executable(marine_sonar_to_pointcloud src/marine_sonar_to_pointcloud.cpp src/ping.cpp)
add_dependencies(marine_sonar_to_pointcloud ${marine_tools_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(marine_sonar_to_pointcloud ${catkin_LIBRARIES})


add_executable(navsatfix_to_nmea src/navsatfix_to_nmea.cpp)
add_dependencies(navsatfix_to_nmea ${marine_tools_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(navsatfix_to_nmea ${catkin_LIBRARIES})

add_executable(imu_to_hdt src/imu_to_hdt.cpp)
add_dependencies(imu_to_hdt ${marine_tools_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(imu_to_hdt ${catkin_LIBRARIES})

install(TARGETS  marine_sonar_to_pointcloud navsatfix_to_nmea imu_to_hdt
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
