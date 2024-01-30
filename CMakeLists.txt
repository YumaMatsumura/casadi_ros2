cmake_minimum_required(VERSION 3.8)
project(casadi_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(casadi REQUIRED)

set(nonlinear_programs_executable nonlinear_programs)
set(nonlinear_programs_library ${nonlinear_programs_executable}_core)
set(dependencies
  rclcpp
  rclcpp_components
  casadi
)

include_directories(
  include
)

add_executable(${nonlinear_programs_executable}
  src/nonlinear_programs_node.cpp
  src/nonlinear_programs_component.cpp
)
add_library(${nonlinear_programs_library} SHARED
  src/nonlinear_programs_component.cpp
)
ament_target_dependencies(${nonlinear_programs_executable}
  ${dependencies}
)
ament_target_dependencies(${nonlinear_programs_library}
  ${dependencies}
)
target_link_libraries(${nonlinear_programs_executable}
  casadi
)
target_link_libraries(${nonlinear_programs_library}
  casadi
)
rclcpp_components_register_nodes(${nonlinear_programs_library}
  "casadi_ros2::NonlinearPrograms"
)

install(TARGETS ${nonlinear_programs_executable}
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS ${nonlinear_programs_library}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
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

ament_export_include_directories(include)
ament_export_libraries(${nonlinear_programs_library})
ament_export_dependencies(${dependencies})
ament_package()
