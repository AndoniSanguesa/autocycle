cmake_minimum_required(VERSION 3.0.2)
project(autocycle)
find_package(catkin REQUIRED COMPONENTS serial message_generation roscpp std_msgs)
catkin_python_setup()
add_message_files(FILES
  Object.msg
  Point.msg
  Curve.msg
)
add_service_files(FILES
  LvxData.srv
  ObjectList.srv
  RollAdj.srv
  GetData.srv
  Action.srv
)
generate_messages(DEPENDENCIES std_msgs)
catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs serial
)
include_directories(include ${catkin_INCLUDE_DIRS})
# PROJECT FILES
add_executable(run_lidar src/run_lidar.cpp)
add_executable(start_nav src/start_nav.cpp)
add_executable(parse_lvx src/parse_lvx.cpp)
add_executable(fix_roll src/fix_roll.cpp)
add_executable(get_data src/get_data.cpp)
add_executable(read_serial src/read_serial.cpp)
add_executable(send_action src/send_action.cpp)

target_link_libraries(run_lidar ${catkin_LIBRARIES})
target_link_libraries(run_lidar stdc++fs)
target_link_libraries(start_nav ${catkin_LIBRARIES})
target_link_libraries(parse_lvx ${catkin_LIBRARIES})
target_link_libraries(fix_roll ${catkin_LIBRARIES})
target_link_libraries(get_data ${catkin_LIBRARIES})
target_link_libraries(read_serial ${catkin_LIBRARIES})
target_link_libraries(send_action ${catkin_LIBRARIES})
catkin_install_python(PROGRAMS bin/bezier_visualization bin/get_deltas
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# TESTING FILES
add_executable(test_roll_adj test/test_roll_adj.cpp)
add_executable(test_roll_publisher test/test_roll_publisher.cpp)
add_executable(test_gen_param test/test_gen_param.cpp)
add_executable(test_visualize_lidar test/test_visualize_lidar.cpp)
add_executable(test_serial_read test/test_serial_read.cpp)
add_executable(test_serial_write test/test_serial_write.cpp)
target_link_libraries(test_roll_adj ${catkin_LIBRARIES})
target_link_libraries(test_roll_publisher ${catkin_LIBRARIES})
target_link_libraries(test_gen_param ${catkin_LIBRARIES})
target_link_libraries(test_visualize_lidar ${catkin_LIBRARIES})
target_link_libraries(test_serial_read ${catkin_LIBRARIES})
target_link_libraries(test_serial_write ${catkin_LIBRARIES})
