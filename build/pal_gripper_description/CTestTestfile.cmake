# CMake generated Testfile for 
# Source directory: /home/lounes_bh/tiago_ws/src/pal_gripper/pal_gripper_description
# Build directory: /home/lounes_bh/tiago_ws/build/pal_gripper_description
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_pal_gripper_description_rostest_test_test_gripper_test.test "/home/lounes_bh/tiago_ws/build/pal_gripper_description/catkin_generated/env_cached.sh" "/usr/bin/python3" "/opt/ros/noetic/share/catkin/cmake/test/run_tests.py" "/home/lounes_bh/tiago_ws/build/pal_gripper_description/test_results/pal_gripper_description/rostest-test_test_gripper_test.xml" "--return-code" "/usr/bin/python3 /opt/ros/noetic/share/rostest/cmake/../../../bin/rostest --pkgdir=/home/lounes_bh/tiago_ws/src/pal_gripper/pal_gripper_description --package=pal_gripper_description --results-filename test_test_gripper_test.xml --results-base-dir \"/home/lounes_bh/tiago_ws/build/pal_gripper_description/test_results\" /home/lounes_bh/tiago_ws/src/pal_gripper/pal_gripper_description/test/test_gripper_test.test ")
set_tests_properties(_ctest_pal_gripper_description_rostest_test_test_gripper_test.test PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/noetic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/noetic/share/rostest/cmake/rostest-extras.cmake;52;catkin_run_tests_target;/home/lounes_bh/tiago_ws/src/pal_gripper/pal_gripper_description/CMakeLists.txt;29;add_rostest;/home/lounes_bh/tiago_ws/src/pal_gripper/pal_gripper_description/CMakeLists.txt;0;")
subdirs("gtest")
