# CMake generated Testfile for 
# Source directory: /home/liu/mir100RL_ws/src/new_mir_gazebo
# Build directory: /home/liu/mir100RL_ws/build/new_mir_gazebo
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_new_mir_gazebo_roslaunch-check_launch "/home/liu/mir100RL_ws/build/new_mir_gazebo/catkin_generated/env_cached.sh" "/usr/bin/python3" "/opt/ros/noetic/share/catkin/cmake/test/run_tests.py" "/home/liu/mir100RL_ws/build/new_mir_gazebo/test_results/new_mir_gazebo/roslaunch-check_launch.xml" "--return-code" "/usr/bin/cmake -E make_directory /home/liu/mir100RL_ws/build/new_mir_gazebo/test_results/new_mir_gazebo" "/opt/ros/noetic/share/roslaunch/cmake/../scripts/roslaunch-check -o \"/home/liu/mir100RL_ws/build/new_mir_gazebo/test_results/new_mir_gazebo/roslaunch-check_launch.xml\" \"/home/liu/mir100RL_ws/src/new_mir_gazebo/launch\" ")
set_tests_properties(_ctest_new_mir_gazebo_roslaunch-check_launch PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/noetic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/noetic/share/roslaunch/cmake/roslaunch-extras.cmake;66;catkin_run_tests_target;/home/liu/mir100RL_ws/src/new_mir_gazebo/CMakeLists.txt;33;roslaunch_add_file_check;/home/liu/mir100RL_ws/src/new_mir_gazebo/CMakeLists.txt;0;")
subdirs("gtest")
