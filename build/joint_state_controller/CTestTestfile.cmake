# CMake generated Testfile for 
# Source directory: /home/lounes_bh/tiago_ws/src/ros_controllers/joint_state_controller
# Build directory: /home/lounes_bh/tiago_ws/build/joint_state_controller
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_joint_state_controller_rostest_test_joint_state_controller.test "/home/lounes_bh/tiago_ws/build/joint_state_controller/catkin_generated/env_cached.sh" "/usr/bin/python3" "/opt/ros/noetic/share/catkin/cmake/test/run_tests.py" "/home/lounes_bh/tiago_ws/build/joint_state_controller/test_results/joint_state_controller/rostest-test_joint_state_controller.xml" "--return-code" "/usr/bin/python3 /opt/ros/noetic/share/rostest/cmake/../../../bin/rostest --pkgdir=/home/lounes_bh/tiago_ws/src/ros_controllers/joint_state_controller --package=joint_state_controller --results-filename test_joint_state_controller.xml --results-base-dir \"/home/lounes_bh/tiago_ws/build/joint_state_controller/test_results\" /home/lounes_bh/tiago_ws/src/ros_controllers/joint_state_controller/test/joint_state_controller.test ")
set_tests_properties(_ctest_joint_state_controller_rostest_test_joint_state_controller.test PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/noetic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/noetic/share/rostest/cmake/rostest-extras.cmake;52;catkin_run_tests_target;/opt/ros/noetic/share/rostest/cmake/rostest-extras.cmake;80;add_rostest;/opt/ros/noetic/share/rostest/cmake/rostest-extras.cmake;100;_add_rostest_google_test;/home/lounes_bh/tiago_ws/src/ros_controllers/joint_state_controller/CMakeLists.txt;22;add_rostest_gtest;/home/lounes_bh/tiago_ws/src/ros_controllers/joint_state_controller/CMakeLists.txt;0;")
subdirs("gtest")
