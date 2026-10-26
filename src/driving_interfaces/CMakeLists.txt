# Created with Claude (Anthropic) on the basis of the car_museum_guide package.
cmake_minimum_required(VERSION 3.8)
project(driving_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MoveToLocation.action"
  "action/VisitorDetected.action"
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # no copyright headers and not linted by cpplint, same as speaking_interfaces
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
