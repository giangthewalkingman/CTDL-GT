# CMake generated Testfile for 
# Source directory: /home/giang/Desktop/cg_enu_yaw_landing_setpoints/src/offboard
# Build directory: /home/giang/Desktop/cg_enu_yaw_landing_setpoints/build/offboard
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_offboard_roslaunch-check_launch "/home/giang/Desktop/cg_enu_yaw_landing_setpoints/build/offboard/catkin_generated/env_cached.sh" "/usr/bin/python3" "/opt/ros/noetic/share/catkin/cmake/test/run_tests.py" "/home/giang/Desktop/cg_enu_yaw_landing_setpoints/build/offboard/test_results/offboard/roslaunch-check_launch.xml" "--return-code" "/usr/bin/cmake -E make_directory /home/giang/Desktop/cg_enu_yaw_landing_setpoints/build/offboard/test_results/offboard" "/opt/ros/noetic/share/roslaunch/cmake/../scripts/roslaunch-check -o \"/home/giang/Desktop/cg_enu_yaw_landing_setpoints/build/offboard/test_results/offboard/roslaunch-check_launch.xml\" \"/home/giang/Desktop/cg_enu_yaw_landing_setpoints/src/offboard/launch\" ")
set_tests_properties(_ctest_offboard_roslaunch-check_launch PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/noetic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/noetic/share/roslaunch/cmake/roslaunch-extras.cmake;66;catkin_run_tests_target;/home/giang/Desktop/cg_enu_yaw_landing_setpoints/src/offboard/CMakeLists.txt;43;roslaunch_add_file_check;/home/giang/Desktop/cg_enu_yaw_landing_setpoints/src/offboard/CMakeLists.txt;0;")
subdirs("gtest")
