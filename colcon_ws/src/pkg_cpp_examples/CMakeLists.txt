cmake_minimum_required(VERSION 3.8)
project(pkg_cpp_examples)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# executable instruction for simple publisher
add_executable(simple_pub src/simple_publisher.cpp)
# declare the dependencies of this executable
ament_target_dependencies(simple_pub rclcpp std_msgs)

# executable instruction for simple subscriber
add_executable(simple_sub src/simple_subscriber.cpp)
# declare the dependencies of this executable
ament_target_dependencies(simple_sub rclcpp std_msgs)

# define install instruction
install(TARGETS
  simple_sub
  DESTINATION lib/${PROJECT_NAME}
)





if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
