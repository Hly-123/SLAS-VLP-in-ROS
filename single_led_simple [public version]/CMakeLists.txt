cmake_minimum_required(VERSION 2.8.3)
project(single_led_simple)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  nav_msgs
  tf
  message_filters
  message_generation
  camera_info_manager
  slo_vlp_msg
  #mvcam
  #serial
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED)
include_directories(include ${OpenCV_INCLUDE_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS})

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   #sensor_angle.msg
   Rect_image.msg
   )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   sensor_msgs
 )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES single_led
CATKIN_DEPENDS cv_bridge geometry_msgs image_transport camera_info_manager roscpp sensor_msgs std_msgs  message_runtime  sensor_msgs#serial #OpenCV REQUIRED
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations



## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/single_led.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/single_led_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )
#--------------------------------------------------------------------
#add_library(imgProcess
#	include/single_led/imgProcess.hpp
#	src/imgProcess.cpp
#)
#--------------------------------------------------------------------

#--------------------------------------------------------------------
add_executable(point_subscriber_simple src/point_subscriber.cpp)
target_link_libraries(point_subscriber_simple ${catkin_LIBRARIES})

add_executable(point_subscriber_in_class_simple src/point_subscriber_in_class.cpp)
target_link_libraries(point_subscriber_in_class_simple ${catkin_LIBRARIES})

add_executable(ekf_translater_simple src/ekf_translater.cpp)
target_link_libraries(ekf_translater_simple ${catkin_LIBRARIES})

add_executable(odom_translater_simple src/odom_translater.cpp)
target_link_libraries(odom_translater_simple ${catkin_LIBRARIES})

add_executable(img_subscriber_simple src/img_subscriber.cpp)
target_link_libraries(img_subscriber_simple ${catkin_LIBRARIES})

target_link_libraries(img_subscriber_simple ${OpenCV_LIBRARIES})

add_executable(imu_subscriber_simple src/imu_subscriber.cpp)
target_link_libraries(imu_subscriber_simple ${catkin_LIBRARIES})



#add_executable(single_led_locator_multi_spin_odom_ROI_from_mvcam
#  src/single_LEDpositioning_opencr9_angle_asyncSpinner_odom_ROI_from_mvcam.cpp
#  src/imgProcess.cpp
#)
#target_link_libraries(single_led_locator_multi_spin_odom_ROI_from_mvcam ${catkin_LIBRARIES})
#target_link_libraries(single_led_locator_multi_spin_odom_ROI_from_mvcam ${OpenCV_LIBRARIES})

#单灯odom+ID解码+ROI跟踪锁定ID
add_executable(single_led_locator_multi_spin_odom_ROI_from_mvcam_ID_decoding_simple
  src/single_LEDpositioning_opencr9_angle_asyncSpinner_odom_ROI_from_mvcam_ID_decoding.cpp
  src/imgProcess_ID.cpp
)
target_link_libraries(single_led_locator_multi_spin_odom_ROI_from_mvcam_ID_decoding_simple ${catkin_LIBRARIES})
target_link_libraries(single_led_locator_multi_spin_odom_ROI_from_mvcam_ID_decoding_simple ${OpenCV_LIBRARIES})
add_dependencies(single_led_locator_multi_spin_odom_ROI_from_mvcam_ID_decoding_simple ${PROJECT_NAME}_generate_messages_cpp)

#单灯imu+ID解码+ROI跟踪锁定ID
add_executable(single_led_locator_multi_spin_imu_ROI_from_mvcam_ID_decoding_simple
  src/single_LEDpositioning_opencr9_angle_asyncSpinner_imu_ROI_from_mvcam_ID_decoding.cpp
  src/imgProcess_ID.cpp
)
target_link_libraries(single_led_locator_multi_spin_imu_ROI_from_mvcam_ID_decoding_simple ${catkin_LIBRARIES})
target_link_libraries(single_led_locator_multi_spin_imu_ROI_from_mvcam_ID_decoding_simple ${OpenCV_LIBRARIES})
add_dependencies(single_led_locator_multi_spin_imu_ROI_from_mvcam_ID_decoding_simple ${PROJECT_NAME}_generate_messages_cpp)

#单灯imu_mag+ID解码+ROI跟踪锁定ID
add_executable(single_led_locator_multi_spin_imu_mag_ROI_from_mvcam_ID_decoding_simple
  src/single_LEDpositioning_opencr9_angle_asyncSpinner_imu_mag_ROI_from_mvcam_ID_decoding.cpp
  src/imgProcess_ID.cpp
)
target_link_libraries(single_led_locator_multi_spin_imu_mag_ROI_from_mvcam_ID_decoding_simple ${catkin_LIBRARIES})
target_link_libraries(single_led_locator_multi_spin_imu_mag_ROI_from_mvcam_ID_decoding_simple ${OpenCV_LIBRARIES})
add_dependencies(single_led_locator_multi_spin_imu_mag_ROI_from_mvcam_ID_decoding_simple ${PROJECT_NAME}_generate_messages_cpp)

#通过粒子滤波融合amcl和vlp的坐标
add_executable(amcl_vlp_fusion
  src/amcl_vlp_fusion.cpp
)
target_link_libraries(amcl_vlp_fusion ${catkin_LIBRARIES})

#单灯正方形灯+ROI跟踪锁定ID
add_executable(single_LEDpositioning_opencr9_angle_asyncSpinner_odom_ROI_from_mvcam_ID_decoding_Square_simple
  src/single_LEDpositioning_opencr9_angle_asyncSpinner_odom_ROI_from_mvcam_ID_decoding_Square.cpp
  src/imgProcess_ID.cpp
)
target_link_libraries(single_LEDpositioning_opencr9_angle_asyncSpinner_odom_ROI_from_mvcam_ID_decoding_Square_simple ${catkin_LIBRARIES})
target_link_libraries(single_LEDpositioning_opencr9_angle_asyncSpinner_odom_ROI_from_mvcam_ID_decoding_Square_simple ${OpenCV_LIBRARIES})
add_dependencies(single_LEDpositioning_opencr9_angle_asyncSpinner_odom_ROI_from_mvcam_ID_decoding_Square_simple ${PROJECT_NAME}_generate_messages_cpp)

####订阅ekf
#add_executable(ekf_output_sub
#  src/ekf_output_sub.cpp
  #src/imgProcess_ID.cpp
#)
#target_link_libraries(ekf_output_sub ${catkin_LIBRARIES})
#target_link_libraries(ekf_output_sub ${OpenCV_LIBRARIES})



#对不同途径获得的角度进行对比(利用松同步)
add_executable(single_led_locator_imu_mag_odom_simple
  src/single_LEDpositioning_opencr9_angle_imu_mag_odom.cpp
  src/imgProcess.cpp
)
target_link_libraries(single_led_locator_imu_mag_odom_simple ${catkin_LIBRARIES})
target_link_libraries(single_led_locator_imu_mag_odom_simple ${OpenCV_LIBRARIES})


#完整功能包应该还有上述程序（即方案一）
#add_executable(stm32_angle_publisher src/stm32_angle_publisher.cpp)
#target_link_libraries(stm32_angle_publisher ${catkin_LIBRARIES})

#add_executable(single_LEDpositioning src/single_LEDpositioning.cpp)
#target_link_libraries(single_LEDpositioning ${catkin_LIBRARIES})
#target_link_libraries(single_LEDpositioning ${OpenCV_LIBRARIES})

#add_definitions(-std=c++17)
#--------------------------------------------------------------------
#方案二，执行此注释掉方案一，两方案均无涉及仅引入头文件的操作
#add_executable(single_LEDpositioning_angle src/single_LEDpositioning_angle.cpp)
#target_link_libraries(single_LEDpositioning_angle imgProcess ${catkin_LIBRARIES})
#target_link_libraries(single_LEDpositioning_angle ${OpenCV_LIBRARIES})
#add_dependencies(single_LEDpositioning_angle ${PROJECT_NAME}_generate_messages_cpp)
#--------------------------------------------------------------------

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )



#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_single_led.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
