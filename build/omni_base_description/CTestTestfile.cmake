# CMake generated Testfile for 
# Source directory: /home/lounes_bh/tiago_ws/src/omni_base_robot/omni_base_description
# Build directory: /home/lounes_bh/tiago_ws/build/omni_base_description
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_omni_base_description_rostest_test_test_omni_base.test "/home/lounes_bh/tiago_ws/build/omni_base_description/catkin_generated/env_cached.sh" "/usr/bin/python3" "/opt/ros/noetic/share/catkin/cmake/test/run_tests.py" "/home/lounes_bh/tiago_ws/build/omni_base_description/test_results/omni_base_description/rostest-test_test_omni_base.xml" "--return-code" "/usr/bin/python3 /opt/ros/noetic/share/rostest/cmake/../../../bin/rostest --pkgdir=/home/lounes_bh/tiago_ws/src/omni_base_robot/omni_base_description --package=omni_base_description --results-filename test_test_omni_base.xml --results-base-dir \"/home/lounes_bh/tiago_ws/build/omni_base_description/test_results\" /home/lounes_bh/tiago_ws/src/omni_base_robot/omni_base_description/test/test_omni_base.test ")
set_tests_properties(_ctest_omni_base_description_rostest_test_test_omni_base.test PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/noetic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/noetic/share/rostest/cmake/rostest-extras.cmake;52;catkin_run_tests_target;/home/lounes_bh/tiago_ws/src/omni_base_robot/omni_base_description/CMakeLists.txt;19;add_rostest;/home/lounes_bh/tiago_ws/src/omni_base_robot/omni_base_description/CMakeLists.txt;0;")
subdirs("gtest")
