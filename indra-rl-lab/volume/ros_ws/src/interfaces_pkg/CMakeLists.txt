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

  "rl_pkg/msg/Pose2D.msg"
  "rl_pkg/msg/Twist2D.msg"
  "rl_pkg/msg/HealthInfo.msg"
  "rl_pkg/msg/LaserScan.msg"
  "rl_pkg/msg/SmartLaserScan.msg"
  "rl_pkg/msg/TriggerSensor.msg"
  "rl_pkg/msg/Turret2DSensor.msg"
  "rl_pkg/msg/Turret2DActuator.msg"

  "use_cases/agents/TankAction.msg"
  "use_cases/agents/TankState.msg"
  "use_cases/agents/EnemyTankState.msg"

  "use_cases/uc1/UC1AgentAction.msg"
  "use_cases/uc1/UC1AgentState.msg"
  "use_cases/uc1/UC1EnvironmentStep.srv"
  "use_cases/uc1/UC1EnvironmentReset.srv"

  "use_cases/uc2/UC2AgentAction.msg"
  "use_cases/uc2/UC2AgentState.msg"
  "use_cases/uc2/UC2EnvironmentStep.srv"
  "use_cases/uc2/UC2EnvironmentReset.srv"

  "use_cases/uc3/UC3AgentAction.msg"
  "use_cases/uc3/UC3AgentState.msg"
  "use_cases/uc3/UC3EnvironmentStep.srv"
  "use_cases/uc3/UC3EnvironmentReset.srv"

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
