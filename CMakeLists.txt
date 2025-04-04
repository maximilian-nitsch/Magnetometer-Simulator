cmake_minimum_required(VERSION 3.8)
project(mag_simulator_package)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(nanoauv_sensor_driver_interfaces REQUIRED)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
        include/mag_simulator_package
        ${EIGEN3_INCLUDE_DIR}
        ${YAML_CPP_INCLUDE_DIR})

set(HEADER_FILES
        include/mag_simulator_package/mag_simulator.h)

add_library(mag_simulator SHARED
        src/mag_simulator.cpp
        ${HEADER_FILES})

# add mag_simulator_node
add_executable(${PROJECT_NAME}_node src/mag_simulator_node.cpp)

ament_target_dependencies(${PROJECT_NAME}_node PUBLIC
        rclcpp
        std_msgs
        sensor_msgs
        nav_msgs
        diagnostic_msgs
        Eigen3
        nanoauv_sensor_driver_interfaces)

target_link_libraries(${PROJECT_NAME}_node PUBLIC mag_simulator)

target_include_directories(${PROJECT_NAME}_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

target_compile_features(${PROJECT_NAME}_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS
        ${PROJECT_NAME}_node
        DESTINATION
        lib/${PROJECT_NAME})

install(DIRECTORY
        launch
        config
        DESTINATION
        share/${PROJECT_NAME}/)

# add csv_test_executable
#add_executable(mag_simulator_static_csv src/mag_simulator_static_csv.cpp)

#ament_target_dependencies(mag_simulator_static_csv PUBLIC
#        Eigen3)

#target_link_libraries(mag_simulator_static_csv PUBLIC mag_simulator)

#target_include_directories(mag_simulator_static_csv PUBLIC
#        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#        $<INSTALL_INTERFACE:include>)

#target_compile_features(mag_simulator_static_csv PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

#install(TARGETS
#        mag_simulator_static_csv
#        DESTINATION lib/${PROJECT_NAME})


install(TARGETS mag_simulator
        EXPORT mag_simulator
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include)

# add odometry test publisher
add_executable(odometry_test_publisher_node src/odometry_test_publisher_node.cpp)

ament_target_dependencies(odometry_test_publisher_node PUBLIC
        rclcpp
        nav_msgs
        tf2_ros
        Eigen3)

target_link_libraries(odometry_test_publisher_node PUBLIC mag_simulator)

target_include_directories(odometry_test_publisher_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

target_compile_features(odometry_test_publisher_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS
        odometry_test_publisher_node
        DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)

  # the following line skips uncrustify (source code formatting)
  # set(ament_cmake_uncrustify_FOUND TRUE)

  find_package(ament_cmake_gtest REQUIRED)

  set(TEST_FILES
      test/main.cpp
      test/mag_simulator_test.cpp)

  ament_add_gtest(${PROJECT_NAME}_test ${TEST_FILES})

  target_link_libraries(${PROJECT_NAME}_test mag_simulator)

  install(TARGETS
          ${PROJECT_NAME}_test
          DESTINATION lib/${PROJECT_NAME})

  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)

  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_uncrustify)

  # enforce linters and static code analyzers defined in ament_lint_common package
  ament_lint_auto_find_test_dependencies()

  # uncomment to include uncrustify explicitly
  # find_package(ament_cmake_uncrustify)
  # ament_uncrustify(CONFIG_FILE "./uncrustify.cfg" TESTNAME "custom_uncrustify")

endif()

ament_package()