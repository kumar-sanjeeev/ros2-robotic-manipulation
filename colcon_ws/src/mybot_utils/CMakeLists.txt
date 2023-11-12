cmake_minimum_required(VERSION 3.8)
project(mybot_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED) # to build python node
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mybot_msgs REQUIRED)
find_package(tf2 REQUIRED)


# to let the complier know we are using python script
ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS
  ${PROJECT_NAME}/angle_conversion.py
  DESTINATION lib/${PROJECT_NAME}
)

# add the following code to make the node as executable (cpp)
add_executable(angle_conversion src/angle_conversion.cpp)
ament_target_dependencies(angle_conversion rclcpp mybot_msgs tf2)

install(TARGETS
    angle_conversion
    DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
