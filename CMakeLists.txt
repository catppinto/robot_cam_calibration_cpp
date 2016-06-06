cmake_minimum_required(VERSION 2.8.3)
project(cat_robotcam_calibration)

find_package(PCL REQUIRED)
find_package(catkin REQUIRED COMPONENTS
roscpp
cat_common
cat_move_to_target
apriltags
tf
geometry_msgs
std_msgs
)
## System dependencies are found with CMake's conventions

catkin_package()

#Build talker and listener
include_directories(include
        ${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        )
link_directories(${PCL_LIBRARY_DIRS})

## Declare a C++ executable
add_executable(robot_camera_calibration src/calibration.cpp)
target_link_libraries(robot_camera_calibration
   ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(fixedTag_calibration src/fixedTagCalibration.cpp src/calculateExtrinsics.cpp)
target_link_libraries(fixedTag_calibration
   ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(runFixedCalibration src/run_fixedCalibration.cpp src/calculateExtrinsics.cpp)
target_link_libraries(runFixedCalibration
   ${catkin_LIBRARIES} ${PCL_LIBRARIES})


add_executable(robot_camera_calibration_new src/new_calibration.cpp)
target_link_libraries(robot_camera_calibration_new
   ${catkin_LIBRARIES} ${PCL_LIBRARIES})


add_executable(transformConverter src/transformConverter.cpp)
target_link_libraries(transformConverter
   ${catkin_LIBRARIES} )

add_executable(gettagpose src/gettagpose_client.cpp)
target_link_libraries(gettagpose
   ${catkin_LIBRARIES} )


