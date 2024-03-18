cmake_minimum_required(VERSION 3.8)
project(nav_client)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
#set(CMAKE_BUILD_TYPE "Debug")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp_components REQUIRED)
#find_package(backward_ros REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
        DIRECTORY
        src
)


#rclcpp_components_register_node(${PROJECT_NAME}
#        PLUGIN nav_client::NavClient
#        EXECUTABLE nav_client_node
#)

rclcpp_components_register_node(${PROJECT_NAME}
        PLUGIN nav_client::VisionClient
        EXECUTABLE vision_client_node
)

#rclcpp_components_register_node(${PROJECT_NAME}
#        PLUGIN nav_client::TestNavSocket
#        EXECUTABLE test_nav_socket_node
#)

rclcpp_components_register_node(${PROJECT_NAME}
        PLUGIN nav_client::UDPSender
        EXECUTABLE test_udp_sender
)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
        INSTALL_TO_SHARE
        config
        launch
)
