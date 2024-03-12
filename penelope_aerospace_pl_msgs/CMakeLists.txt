cmake_minimum_required(VERSION 3.8)
project(penelope_aerospace_pl_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AssemblyAction.msg"
  "msg/AssemblyActionState.msg"
  "msg/AssemblyDrill.msg"
  "msg/AssemblyEe.msg"
  "msg/AssemblyEeDockingPos.msg"
  "msg/AssemblyEeState.msg"
  "msg/AssemblyFast.msg"
  "msg/AssemblyFastState.msg"
  "msg/AssemblyHoleLocation.msg"
  "msg/AssemblyHoleLocationContainer.msg"
  "msg/AssemblyMaterialLayer.msg"
  "msg/AssemblyWaypoint.msg"
  "msg/ModuleState.msg"
  "msg/ResultCodes.msg"
  "action/CobotOp.action"

  DEPENDENCIES # Add packages that above messages depend on, in this case geometry_msgs for Sphere.msg
    geometry_msgs
    shape_msgs
    sensor_msgs
    trajectory_msgs
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

ament_package()
