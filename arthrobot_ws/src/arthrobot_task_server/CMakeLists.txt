cmake_minimum_required(VERSION 3.8)
project(arthrobot_task_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(arthrobot_interfaces)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(Eigen3 REQUIRED)

# Arthrobot Task Server
add_library(arthrobot_task_server SHARED src/arthrobot_task_server.cpp)
target_include_directories(arthrobot_task_server PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)

target_compile_definitions(arthrobot_task_server PRIVATE "ARTHROBOT_TASK_SERVER_CPP_BUILDING_DLL")
ament_target_dependencies(arthrobot_task_server
                                                "arthrobot_interfaces"
                                                "rclcpp"
                                                "rclcpp_action"
                                                "rclcpp_components"
                                                "moveit_ros_planning_interface")
rclcpp_components_register_node(arthrobot_task_server PLUGIN "arthrobot_task_server_node::ArthrobotTaskServerNode" EXECUTABLE arthrobot_task_server_node)

install(TARGETS 
  arthrobot_task_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
