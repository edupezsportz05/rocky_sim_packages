cmake_minimum_required(VERSION 3.8)
project(my_robot_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- Dependencies (needed even without custom C++ nodes, for consistency/linting) ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_common REQUIRED)

# --- Install directories so they land in share/my_robot_navigation/ at build time ---
install(DIRECTORY
  launch
  config
  maps
  rviz
  behavior_trees
  DESTINATION share/${PROJECT_NAME}/
)

# --- Install the include/ headers, in case you add C++ nodes later ---
install(DIRECTORY
  include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

# --- Install python scripts ---
install(PROGRAMS
  scripts/scan_filter.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
