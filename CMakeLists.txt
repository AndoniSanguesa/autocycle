cmake_minimum_required(VERSION 3.0.2)
project(autocycle)
find_package(catkin REQUIRED COMPONENTS autocycle_extras serial roscpp std_msgs)
catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS  std_msgs serial
)
include_directories(include ${catkin_INCLUDE_DIRS})
# PROJECT FILES
add_executable(run_lidar src/run_lidar.cpp)
add_executable(start_nav src/start_nav.cpp)
add_executable(frame_ready src/frame_ready.cpp)
add_executable(read_serial src/read_serial.cpp)
add_executable(send_action src/send_action.cpp)

add_dependencies(run_lidar ${catkin_EXPORTED_TARGETS})
add_dependencies(start_nav ${catkin_EXPORTED_TARGETS})
add_dependencies(frame_ready ${catkin_EXPORTED_TARGETS})
add_dependencies(read_serial ${catkin_EXPORTED_TARGETS})
add_dependencies(send_action ${catkin_EXPORTED_TARGETS})

target_link_libraries(run_lidar stdc++fs)
target_link_libraries(run_lidar ${catkin_LIBRARIES})
target_link_libraries(start_nav ${catkin_LIBRARIES})
target_link_libraries(frame_ready ${catkin_LIBRARIES})
target_link_libraries(read_serial ${catkin_LIBRARIES})
target_link_libraries(send_action ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS bin/read_rosbag bin/get_deltas
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# TESTING FILES
add_executable(test_bezier test/test_bezier.cpp)
add_executable(test_serial_read test/test_serial_read.cpp)
add_executable(test_sim_sensors test/test_sim_sensors.cpp)
add_executable(test_serial_write test/test_serial_write.cpp)
add_executable(test_fake_send_action test/test_fake_send_action.cpp)

target_link_libraries(test_bezier ${catkin_LIBRARIES})
target_link_libraries(test_serial_read ${catkin_LIBRARIES})
target_link_libraries(test_sim_sensors ${catkin_LIBRARIES})
target_link_libraries(test_serial_write ${catkin_LIBRARIES})
target_link_libraries(test_fake_send_action ${catkin_LIBRARIES})
