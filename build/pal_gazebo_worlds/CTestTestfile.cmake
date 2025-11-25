# CMake generated Testfile for 
# Source directory: /home/lounes_bh/tiago_ws/src/pal_gazebo_worlds
# Build directory: /home/lounes_bh/tiago_ws/build/pal_gazebo_worlds
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_pal_gazebo_worlds_rostest_test_run_gazebo_server.test "/home/lounes_bh/tiago_ws/build/pal_gazebo_worlds/catkin_generated/env_cached.sh" "/usr/bin/python3" "/opt/ros/noetic/share/catkin/cmake/test/run_tests.py" "/home/lounes_bh/tiago_ws/build/pal_gazebo_worlds/test_results/pal_gazebo_worlds/rostest-test_run_gazebo_server.xml" "--return-code" "/usr/bin/python3 /opt/ros/noetic/share/rostest/cmake/../../../bin/rostest --pkgdir=/home/lounes_bh/tiago_ws/src/pal_gazebo_worlds --package=pal_gazebo_worlds --results-filename test_run_gazebo_server.xml --results-base-dir \"/home/lounes_bh/tiago_ws/build/pal_gazebo_worlds/test_results\" /home/lounes_bh/tiago_ws/src/pal_gazebo_worlds/test/run_gazebo_server.test ")
set_tests_properties(_ctest_pal_gazebo_worlds_rostest_test_run_gazebo_server.test PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/noetic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/noetic/share/rostest/cmake/rostest-extras.cmake;52;catkin_run_tests_target;/opt/ros/noetic/share/rostest/cmake/rostest-extras.cmake;80;add_rostest;/opt/ros/noetic/share/rostest/cmake/rostest-extras.cmake;100;_add_rostest_google_test;/home/lounes_bh/tiago_ws/src/pal_gazebo_worlds/CMakeLists.txt;34;add_rostest_gtest;/home/lounes_bh/tiago_ws/src/pal_gazebo_worlds/CMakeLists.txt;0;")
subdirs("gtest")
