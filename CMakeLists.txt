cmake_minimum_required(VERSION 3.8)
project(rebet_mirte)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies We import external dependencies for both Cpp and Python at the same time.
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclpy REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(rebet_msgs REQUIRED)
find_package(rebet REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(slam_toolbox REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(aal_msgs REQUIRED)

set(THIS_PACKAGE_DEPS
    rclcpp
    rclcpp_lifecycle
    behaviortree_ros2
    behaviortree_cpp
    rebet_msgs
    lifecycle_msgs
    diagnostic_msgs
    geometry_msgs
    nav_msgs
    nav2_msgs
    sensor_msgs
    rebet
    slam_toolbox
    aal_msgs)

# Include Cpp "include" directory
include_directories(include)
# Create Cpp executable
add_executable(mirte_arborist src/mirte_arborist.cpp)
ament_target_dependencies(mirte_arborist ${THIS_PACKAGE_DEPS})

add_library(navigate_to_pose_plugin SHARED src/navigatetopose_action.cpp)
target_compile_definitions(navigate_to_pose_plugin PRIVATE  BT_PLUGIN_EXPORT)
ament_target_dependencies(navigate_to_pose_plugin ${THIS_PACKAGE_DEPS})

# Install Cpp executables
install(TARGETS
  mirte_arborist
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/system_reflection.py
  scripts/tree_action_client.py
  scripts/context_model.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  navigate_to_pose_plugin
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  navigate_to_pose_plugin
  LIBRARY DESTINATION share/${PROJECT_NAME}/bt_plugins
  ARCHIVE DESTINATION share/${PROJECT_NAME}/bt_plugins
  RUNTIME DESTINATION share/${PROJECT_NAME}/bt_plugins
)


install(DIRECTORY
  launch
  config
  trees
  models
  worlds
  DESTINATION share/${PROJECT_NAME}/
)


if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # ament_add_gtest(${PROJECT_NAME}_tutorial_test test/tutorial_test.cpp)
  # target_include_directories(${PROJECT_NAME}_tutorial_test PUBLIC
  #   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  #   $<INSTALL_INTERFACE:include>
  # )

 # Integration tests
#  find_package(ament_cmake_ros REQUIRED)
#  find_package(launch_testing_ament_cmake REQUIRED)
#  function(add_ros_isolated_launch_test path)
#    set(RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py")
#    add_launch_test("${path}" RUNNER "${RUNNER}" ${ARGN})
#  endfunction()
#  add_ros_isolated_launch_test(test/test_launch.py)

  # target_link_libraries(${PROJECT_NAME}_tutorial_test name_of_local_library)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  # set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
