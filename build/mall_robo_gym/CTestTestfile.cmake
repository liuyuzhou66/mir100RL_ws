# CMake generated Testfile for 
# Source directory: /home/dfki.uni-bremen.de/abresser/mir100RL_ws/src/mall_robo_gym
# Build directory: /home/dfki.uni-bremen.de/abresser/mir100RL_ws/build/mall_robo_gym
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_mall_robo_gym_roslaunch-check_launch "/home/dfki.uni-bremen.de/abresser/mir100RL_ws/build/mall_robo_gym/catkin_generated/env_cached.sh" "/usr/bin/python3" "/opt/ros/noetic/share/catkin/cmake/test/run_tests.py" "/home/dfki.uni-bremen.de/abresser/mir100RL_ws/build/mall_robo_gym/test_results/mall_robo_gym/roslaunch-check_launch.xml" "--return-code" "/usr/bin/cmake -E make_directory /home/dfki.uni-bremen.de/abresser/mir100RL_ws/build/mall_robo_gym/test_results/mall_robo_gym" "/opt/ros/noetic/share/roslaunch/cmake/../scripts/roslaunch-check -o \"/home/dfki.uni-bremen.de/abresser/mir100RL_ws/build/mall_robo_gym/test_results/mall_robo_gym/roslaunch-check_launch.xml\" \"/home/dfki.uni-bremen.de/abresser/mir100RL_ws/src/mall_robo_gym/launch\" ")
set_tests_properties(_ctest_mall_robo_gym_roslaunch-check_launch PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/noetic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/noetic/share/roslaunch/cmake/roslaunch-extras.cmake;66;catkin_run_tests_target;/home/dfki.uni-bremen.de/abresser/mir100RL_ws/src/mall_robo_gym/CMakeLists.txt;29;roslaunch_add_file_check;/home/dfki.uni-bremen.de/abresser/mir100RL_ws/src/mall_robo_gym/CMakeLists.txt;0;")
subdirs("gtest")
