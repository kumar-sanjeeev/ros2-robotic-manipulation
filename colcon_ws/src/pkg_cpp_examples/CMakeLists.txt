cmake_minimum_required(VERSION 3.8)
project(pkg_cpp_examples)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(mybot_msgs REQUIRED)



# executable instruction for simple publisher
add_executable(simple_pub src/simple_publisher.cpp)
# declare the dependencies of this executable
ament_target_dependencies(simple_pub rclcpp std_msgs)

# executable instruction for simple subscriber
add_executable(simple_sub src/simple_subscriber.cpp)
# declare the dependencies of this executable
ament_target_dependencies(simple_sub rclcpp std_msgs)

# executable instruction for simple parameter
add_executable(simple_param src/simple_parameters.cpp)
# declare the dependencies of this executable
ament_target_dependencies(simple_param rclcpp rcl_interfaces)

# executable instruction for simple service server
add_executable(simple_service_server src/simple_service_server.cpp)
# declare the dependencies of this executable
ament_target_dependencies(simple_service_server rclcpp mybot_msgs)

# executable instruction for simple service client
add_executable(simple_service_client src/simple_service_client.cpp)
# declare the dependencies of this executable
ament_target_dependencies(simple_service_client rclcpp mybot_msgs)

# define install instruction
install(TARGETS
  simple_sub
  simple_pub
  simple_param
  simple_service_server
  simple_service_client
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
