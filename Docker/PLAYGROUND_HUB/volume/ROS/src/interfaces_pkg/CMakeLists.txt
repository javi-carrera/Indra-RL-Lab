cmake_minimum_required(VERSION 3.8)
project(interfaces_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs  REQUIRED)
find_package(sensor_msgs  REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}

  "playground_pkg/msg/TriggerSensor.msg"
  "playground_pkg/msg/SmartLidarSensor.msg"
  "playground_pkg/msg/EnemySensor.msg"
  "playground_pkg/msg/Turret2DSensor.msg"
  "playground_pkg/msg/Turret2DActuator.msg"

  "examples_pkg/msg/AutonomousNavigationExampleAgentAction.msg"
  "examples_pkg/msg/AutonomousNavigationExampleAgentState.msg"
  "examples_pkg/msg/AutonomousNavigationExampleAgentReset.msg"
  "examples_pkg/srv/AutonomousNavigationExampleEnvironmentStep.srv"
  "examples_pkg/srv/AutonomousNavigationExampleEnvironmentReset.srv"

  "examples_pkg/msg/ShootingExampleAgentAction.msg"
  "examples_pkg/msg/ShootingExampleAgentState.msg"
  "examples_pkg/msg/ShootingExampleAgentReset.msg"
  "examples_pkg/srv/ShootingExampleEnvironmentStep.srv"
  "examples_pkg/srv/ShootingExampleEnvironmentReset.srv"

  DEPENDENCIES geometry_msgs
  DEPENDENCIES sensor_msgs

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
