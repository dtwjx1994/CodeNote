# CMake generated Testfile for 
# Source directory: /home/sc/文档/codenote/laser_scan_matcher
# Build directory: /home/sc/文档/codenote/laser_scan_matcher/cmake-build-debug
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_laser_scan_matcher_rostest_test_run.test "/home/sc/文档/codenote/laser_scan_matcher/cmake-build-debug/catkin_generated/env_cached.sh" "/usr/bin/python" "/opt/ros/kinetic/share/catkin/cmake/test/run_tests.py" "/home/sc/文档/codenote/laser_scan_matcher/cmake-build-debug/test_results/laser_scan_matcher/rostest-test_run.xml" "--return-code" "/opt/ros/kinetic/share/rostest/cmake/../../../bin/rostest --pkgdir=/home/sc/文档/codenote/laser_scan_matcher --package=laser_scan_matcher --results-filename test_run.xml --results-base-dir \"/home/sc/文档/codenote/laser_scan_matcher/cmake-build-debug/test_results\" /home/sc/文档/codenote/laser_scan_matcher/test/run.test ")
add_test(_ctest_laser_scan_matcher_rostest_test_covariance.test "/home/sc/文档/codenote/laser_scan_matcher/cmake-build-debug/catkin_generated/env_cached.sh" "/usr/bin/python" "/opt/ros/kinetic/share/catkin/cmake/test/run_tests.py" "/home/sc/文档/codenote/laser_scan_matcher/cmake-build-debug/test_results/laser_scan_matcher/rostest-test_covariance.xml" "--return-code" "/opt/ros/kinetic/share/rostest/cmake/../../../bin/rostest --pkgdir=/home/sc/文档/codenote/laser_scan_matcher --package=laser_scan_matcher --results-filename test_covariance.xml --results-base-dir \"/home/sc/文档/codenote/laser_scan_matcher/cmake-build-debug/test_results\" /home/sc/文档/codenote/laser_scan_matcher/test/covariance.test ")
subdirs("gtest")
