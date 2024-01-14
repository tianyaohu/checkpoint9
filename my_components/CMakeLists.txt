cmake_minimum_required(VERSION 3.8)
project(my_components)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(composition REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(attach_shelf REQUIRED)

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

#include
include_directories(include)

################## pre approach component ################## 
add_library(pre_approach_component SHARED src/pre_approach.cpp)
target_compile_definitions(pre_approach_component PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(pre_approach_component
  "rclcpp"
  "rclcpp_components"
  "nav_msgs"
  "sensor_msgs"
  "geometry_msgs"
  )
rclcpp_components_register_nodes(pre_approach_component "my_components::PreApproach")
set(node_plugins "${node_plugins}my_components::PreApproach;$<TARGET_FILE:pre_approach_component>\n")


################## Attach Server Component ################## 
add_library(attach_server_component SHARED src/attach_server.cpp)
target_compile_definitions(attach_server_component PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(attach_server_component
  "rclcpp"
  "rclcpp_components"
  "std_msgs"
  "sensor_msgs"
  "geometry_msgs"
  "tf2_ros"
  "tf2_ros"
  "tf2"
  "attach_shelf")
rclcpp_components_register_nodes(attach_server_component "my_components::AttachServer")
set(node_plugins "${node_plugins}my_components::AttachServer;$<TARGET_FILE:attach_server_component>\n")

################## MANUAL COMPOSITION of server ################## 
add_executable(attach_server_manual_composition
  src/attach_server_manual_composition.cpp)
target_link_libraries(attach_server_manual_composition
  attach_server_component)
ament_target_dependencies(attach_server_manual_composition
  "rclcpp")

install(TARGETS
  attach_server_manual_composition
  DESTINATION lib/${PROJECT_NAME})

################## Attach Client ################## 
add_library(attach_client_component SHARED src/attach_client.cpp)
target_compile_definitions(attach_client_component PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(attach_client_component
  "rclcpp"
  "rclcpp_components"
   "attach_shelf"
  )
rclcpp_components_register_nodes(attach_client_component "my_components::AttachClient")
set(node_plugins "${node_plugins}my_components::AttachClient;$<TARGET_FILE:attach_client_component>\n")

#install targets
install(TARGETS
  pre_approach_component
  attach_server_component
  attach_server_manual_composition
  attach_client_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Install dirs
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
ament_package()
