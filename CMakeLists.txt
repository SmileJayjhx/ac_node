cmake_minimum_required(VERSION 3.0.2)
project(ac_node)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  common
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  config/Servo.cfg
  #...
)

## Generate messages
catkin_package(
  INCLUDE_DIRS 
#  LIBRARIES learning_communication
  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# add_executable(ac_node src/packag.cpp src/servo_driver.cpp)
# target_link_libraries(ac_node ${catkin_LIBRARIES})
# add_dependencies(ac_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


# add_executable(ac_mima_node src/main_mima.cpp src/driver/mima_driver.cpp)
# target_link_libraries(ac_mima_node ${catkin_LIBRARIES})
# add_dependencies(ac_mima_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# add_executable(ac_rebot_node src/main_rebot.cpp  src/driver/rebot_driver.cpp)
# target_link_libraries(ac_rebot_node ${catkin_LIBRARIES})
# add_dependencies(ac_rebot_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# add_executable(ac_ruyi_old_node src/main_ruyi_old.cpp src/driver/servo_driver_ruyi_old.cpp src/driver/curtis_driver.cpp)
# target_link_libraries(ac_ruyi_old_node ${catkin_LIBRARIES})
# add_dependencies(ac_ruyi_old_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# add_executable(ac_ruyi_omv_node src/main_ruyi_omv.cpp src/driver/servo_driver.cpp src/driver/curtis_driver.cpp)
# target_link_libraries(ac_ruyi_omv_node ${catkin_LIBRARIES})
# add_dependencies(ac_ruyi_omv_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# add_executable(ac_ruyi_single_steering_node src/main_ruyi_single_steering.cpp  src/driver/curtis_driver.cpp)
# target_link_libraries(ac_ruyi_single_steering_node ${catkin_LIBRARIES})
# add_dependencies(ac_ruyi_single_steering_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_executable(main_ac_node src/main_ac.cpp  src/driver/curtis_driver.cpp src/driver/rebot_driver.cpp src/driver/mima_driver.cpp src/driver/servo_driver.cpp src/driver/YJL_driver.cpp)
target_link_libraries(main_ac_node ${catkin_LIBRARIES})
add_dependencies(main_ac_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(ac_driver_manager src/ac_driver_manager.cpp src/driver/curtis_driver.cpp src/driver/rebot_driver.cpp src/driver/mima_driver.cpp src/driver/servo_driver.cpp src/driver/YJL_driver.cpp)
target_link_libraries(ac_driver_manager ${catkin_LIBRARIES})
add_dependencies(ac_driver_manager ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# install(TARGETS ac_mima_node
# ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# install(TARGETS ac_rebot_node
# ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# install(TARGETS ac_ruyi_old_node
# ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# install(TARGETS ac_ruyi_omv_node
# ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# install(TARGETS ac_ruyi_single_steering_node
# ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

install(TARGETS main_ac_node
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS ac_driver_manager
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
FILES_MATCHING PATTERN "*.h"
PATTERN ".svn" EXCLUDE
)

install(DIRECTORY config
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)