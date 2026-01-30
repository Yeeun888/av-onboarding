cmake_minimum_required(VERSION 3.8)
project(cipher_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cipher_interfaces REQUIRED)

add_executable(cipher_node src/cipher_node.cpp)
ament_target_dependencies(cipher_node rclcpp cipher_interfaces)

add_executable(participant_node src/participant_node.cpp)
ament_target_dependencies(participant_node rclcpp cipher_interfaces)

install(TARGETS
  cipher_node
  participant_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
