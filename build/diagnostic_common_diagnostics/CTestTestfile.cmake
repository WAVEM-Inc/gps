# CMake generated Testfile for 
# Source directory: /home/ubuntu/ros2_ws/src/gps/diagnostics/diagnostic_common_diagnostics
# Build directory: /home/ubuntu/ros2_ws/src/gps/build/diagnostic_common_diagnostics
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(test_cpu_monitor "/usr/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/ubuntu/ros2_ws/src/gps/build/diagnostic_common_diagnostics/test_results/diagnostic_common_diagnostics/test_cpu_monitor.xunit.xml" "--package-name" "diagnostic_common_diagnostics" "--output-file" "/home/ubuntu/ros2_ws/src/gps/build/diagnostic_common_diagnostics/ament_cmake_pytest/test_cpu_monitor.txt" "--command" "/usr/bin/python3" "-u" "-m" "pytest" "/home/ubuntu/ros2_ws/src/gps/diagnostics/diagnostic_common_diagnostics/test/systemtest/test_cpu_monitor.py" "-o" "cache_dir=/home/ubuntu/ros2_ws/src/gps/build/diagnostic_common_diagnostics/ament_cmake_pytest/test_cpu_monitor/.cache" "--junit-xml=/home/ubuntu/ros2_ws/src/gps/build/diagnostic_common_diagnostics/test_results/diagnostic_common_diagnostics/test_cpu_monitor.xunit.xml" "--junit-prefix=diagnostic_common_diagnostics")
set_tests_properties(test_cpu_monitor PROPERTIES  LABELS "pytest" TIMEOUT "10" WORKING_DIRECTORY "/home/ubuntu/ros2_ws/src/gps/build/diagnostic_common_diagnostics" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_pytest/cmake/ament_add_pytest_test.cmake;169;ament_add_test;/home/ubuntu/ros2_ws/src/gps/diagnostics/diagnostic_common_diagnostics/CMakeLists.txt;22;ament_add_pytest_test;/home/ubuntu/ros2_ws/src/gps/diagnostics/diagnostic_common_diagnostics/CMakeLists.txt;0;")
add_test(ntp_monitor_launchtest "/usr/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/ubuntu/ros2_ws/src/gps/build/diagnostic_common_diagnostics/test_results/diagnostic_common_diagnostics/ntp_monitor_launchtest.xunit.xml" "--package-name" "diagnostic_common_diagnostics" "--output-file" "/home/ubuntu/ros2_ws/src/gps/build/diagnostic_common_diagnostics/launch_test/ntp_monitor_launchtest.txt" "--command" "/usr/bin/python3" "-m" "launch_testing.launch_test" "/home/ubuntu/ros2_ws/src/gps/diagnostics/diagnostic_common_diagnostics/test/systemtest/test_ntp_monitor_launchtest.py" "--junit-xml=/home/ubuntu/ros2_ws/src/gps/build/diagnostic_common_diagnostics/test_results/diagnostic_common_diagnostics/ntp_monitor_launchtest.xunit.xml" "--package-name=diagnostic_common_diagnostics")
set_tests_properties(ntp_monitor_launchtest PROPERTIES  LABELS "launch_test" TIMEOUT "20" WORKING_DIRECTORY "/home/ubuntu/ros2_ws/src/gps/build/diagnostic_common_diagnostics" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/launch_testing_ament_cmake/cmake/add_launch_test.cmake;131;ament_add_test;/home/ubuntu/ros2_ws/src/gps/diagnostics/diagnostic_common_diagnostics/CMakeLists.txt;26;add_launch_test;/home/ubuntu/ros2_ws/src/gps/diagnostics/diagnostic_common_diagnostics/CMakeLists.txt;0;")
add_test(xmllint "/usr/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/ubuntu/ros2_ws/src/gps/build/diagnostic_common_diagnostics/test_results/diagnostic_common_diagnostics/xmllint.xunit.xml" "--package-name" "diagnostic_common_diagnostics" "--output-file" "/home/ubuntu/ros2_ws/src/gps/build/diagnostic_common_diagnostics/ament_xmllint/xmllint.txt" "--command" "/opt/ros/humble/bin/ament_xmllint" "--xunit-file" "/home/ubuntu/ros2_ws/src/gps/build/diagnostic_common_diagnostics/test_results/diagnostic_common_diagnostics/xmllint.xunit.xml")
set_tests_properties(xmllint PROPERTIES  LABELS "xmllint;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/ubuntu/ros2_ws/src/gps/diagnostics/diagnostic_common_diagnostics" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_xmllint/cmake/ament_xmllint.cmake;50;ament_add_test;/opt/ros/humble/share/ament_cmake_xmllint/cmake/ament_cmake_xmllint_lint_hook.cmake;18;ament_xmllint;/opt/ros/humble/share/ament_cmake_xmllint/cmake/ament_cmake_xmllint_lint_hook.cmake;0;;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/humble/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;21;ament_execute_extensions;/opt/ros/humble/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;0;;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_package.cmake;66;ament_execute_extensions;/home/ubuntu/ros2_ws/src/gps/diagnostics/diagnostic_common_diagnostics/CMakeLists.txt;32;ament_package;/home/ubuntu/ros2_ws/src/gps/diagnostics/diagnostic_common_diagnostics/CMakeLists.txt;0;")
add_test(lint_cmake "/usr/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/ubuntu/ros2_ws/src/gps/build/diagnostic_common_diagnostics/test_results/diagnostic_common_diagnostics/lint_cmake.xunit.xml" "--package-name" "diagnostic_common_diagnostics" "--output-file" "/home/ubuntu/ros2_ws/src/gps/build/diagnostic_common_diagnostics/ament_lint_cmake/lint_cmake.txt" "--command" "/opt/ros/humble/bin/ament_lint_cmake" "--xunit-file" "/home/ubuntu/ros2_ws/src/gps/build/diagnostic_common_diagnostics/test_results/diagnostic_common_diagnostics/lint_cmake.xunit.xml")
set_tests_properties(lint_cmake PROPERTIES  LABELS "lint_cmake;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/ubuntu/ros2_ws/src/gps/diagnostics/diagnostic_common_diagnostics" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_lint_cmake/cmake/ament_lint_cmake.cmake;47;ament_add_test;/opt/ros/humble/share/ament_cmake_lint_cmake/cmake/ament_cmake_lint_cmake_lint_hook.cmake;21;ament_lint_cmake;/opt/ros/humble/share/ament_cmake_lint_cmake/cmake/ament_cmake_lint_cmake_lint_hook.cmake;0;;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/humble/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;21;ament_execute_extensions;/opt/ros/humble/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;0;;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_package.cmake;66;ament_execute_extensions;/home/ubuntu/ros2_ws/src/gps/diagnostics/diagnostic_common_diagnostics/CMakeLists.txt;32;ament_package;/home/ubuntu/ros2_ws/src/gps/diagnostics/diagnostic_common_diagnostics/CMakeLists.txt;0;")
