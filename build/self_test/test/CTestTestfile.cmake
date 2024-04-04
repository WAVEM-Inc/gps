# CMake generated Testfile for 
# Source directory: /home/ubuntu/ros2_ws/src/gps/diagnostics/self_test/test
# Build directory: /home/ubuntu/ros2_ws/src/gps/build/self_test/test
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(test_nominal_selftest "/usr/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/ubuntu/ros2_ws/src/gps/build/self_test/test_results/self_test/test_nominal_selftest.gtest.xml" "--package-name" "self_test" "--output-file" "/home/ubuntu/ros2_ws/src/gps/build/self_test/ament_cmake_gtest/test_nominal_selftest.txt" "--command" "/home/ubuntu/ros2_ws/src/gps/build/self_test/test/test_nominal_selftest" "--gtest_output=xml:/home/ubuntu/ros2_ws/src/gps/build/self_test/test_results/self_test/test_nominal_selftest.gtest.xml")
set_tests_properties(test_nominal_selftest PROPERTIES  LABELS "gtest" REQUIRED_FILES "/home/ubuntu/ros2_ws/src/gps/build/self_test/test/test_nominal_selftest" TIMEOUT "60" WORKING_DIRECTORY "/home/ubuntu/ros2_ws/src/gps/build/self_test/test" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_gtest/cmake/ament_add_gtest_test.cmake;86;ament_add_test;/opt/ros/humble/share/ament_cmake_gtest/cmake/ament_add_gtest.cmake;93;ament_add_gtest_test;/home/ubuntu/ros2_ws/src/gps/diagnostics/self_test/test/CMakeLists.txt;4;ament_add_gtest;/home/ubuntu/ros2_ws/src/gps/diagnostics/self_test/test/CMakeLists.txt;20;add_self_test;/home/ubuntu/ros2_ws/src/gps/diagnostics/self_test/test/CMakeLists.txt;0;")
add_test(test_no_id_selftest "/usr/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/ubuntu/ros2_ws/src/gps/build/self_test/test_results/self_test/test_no_id_selftest.gtest.xml" "--package-name" "self_test" "--output-file" "/home/ubuntu/ros2_ws/src/gps/build/self_test/ament_cmake_gtest/test_no_id_selftest.txt" "--command" "/home/ubuntu/ros2_ws/src/gps/build/self_test/test/test_no_id_selftest" "--gtest_output=xml:/home/ubuntu/ros2_ws/src/gps/build/self_test/test_results/self_test/test_no_id_selftest.gtest.xml")
set_tests_properties(test_no_id_selftest PROPERTIES  LABELS "gtest" REQUIRED_FILES "/home/ubuntu/ros2_ws/src/gps/build/self_test/test/test_no_id_selftest" TIMEOUT "60" WORKING_DIRECTORY "/home/ubuntu/ros2_ws/src/gps/build/self_test/test" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_gtest/cmake/ament_add_gtest_test.cmake;86;ament_add_test;/opt/ros/humble/share/ament_cmake_gtest/cmake/ament_add_gtest.cmake;93;ament_add_gtest_test;/home/ubuntu/ros2_ws/src/gps/diagnostics/self_test/test/CMakeLists.txt;4;ament_add_gtest;/home/ubuntu/ros2_ws/src/gps/diagnostics/self_test/test/CMakeLists.txt;21;add_self_test;/home/ubuntu/ros2_ws/src/gps/diagnostics/self_test/test/CMakeLists.txt;0;")
add_test(test_exception_selftest "/usr/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/ubuntu/ros2_ws/src/gps/build/self_test/test_results/self_test/test_exception_selftest.gtest.xml" "--package-name" "self_test" "--output-file" "/home/ubuntu/ros2_ws/src/gps/build/self_test/ament_cmake_gtest/test_exception_selftest.txt" "--command" "/home/ubuntu/ros2_ws/src/gps/build/self_test/test/test_exception_selftest" "--gtest_output=xml:/home/ubuntu/ros2_ws/src/gps/build/self_test/test_results/self_test/test_exception_selftest.gtest.xml")
set_tests_properties(test_exception_selftest PROPERTIES  LABELS "gtest" REQUIRED_FILES "/home/ubuntu/ros2_ws/src/gps/build/self_test/test/test_exception_selftest" TIMEOUT "60" WORKING_DIRECTORY "/home/ubuntu/ros2_ws/src/gps/build/self_test/test" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_gtest/cmake/ament_add_gtest_test.cmake;86;ament_add_test;/opt/ros/humble/share/ament_cmake_gtest/cmake/ament_add_gtest.cmake;93;ament_add_gtest_test;/home/ubuntu/ros2_ws/src/gps/diagnostics/self_test/test/CMakeLists.txt;4;ament_add_gtest;/home/ubuntu/ros2_ws/src/gps/diagnostics/self_test/test/CMakeLists.txt;22;add_self_test;/home/ubuntu/ros2_ws/src/gps/diagnostics/self_test/test/CMakeLists.txt;0;")
add_test(test_error_selftest "/usr/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/ubuntu/ros2_ws/src/gps/build/self_test/test_results/self_test/test_error_selftest.gtest.xml" "--package-name" "self_test" "--output-file" "/home/ubuntu/ros2_ws/src/gps/build/self_test/ament_cmake_gtest/test_error_selftest.txt" "--command" "/home/ubuntu/ros2_ws/src/gps/build/self_test/test/test_error_selftest" "--gtest_output=xml:/home/ubuntu/ros2_ws/src/gps/build/self_test/test_results/self_test/test_error_selftest.gtest.xml")
set_tests_properties(test_error_selftest PROPERTIES  LABELS "gtest" REQUIRED_FILES "/home/ubuntu/ros2_ws/src/gps/build/self_test/test/test_error_selftest" TIMEOUT "60" WORKING_DIRECTORY "/home/ubuntu/ros2_ws/src/gps/build/self_test/test" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_gtest/cmake/ament_add_gtest_test.cmake;86;ament_add_test;/opt/ros/humble/share/ament_cmake_gtest/cmake/ament_add_gtest.cmake;93;ament_add_gtest_test;/home/ubuntu/ros2_ws/src/gps/diagnostics/self_test/test/CMakeLists.txt;4;ament_add_gtest;/home/ubuntu/ros2_ws/src/gps/diagnostics/self_test/test/CMakeLists.txt;23;add_self_test;/home/ubuntu/ros2_ws/src/gps/diagnostics/self_test/test/CMakeLists.txt;0;")
subdirs("../gtest")
