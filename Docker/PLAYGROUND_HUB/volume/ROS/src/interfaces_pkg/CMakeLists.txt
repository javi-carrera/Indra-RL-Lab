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

  "examples_pkg/msg/AutonomousNavigationExampleAgentAction.msg"
  "examples_pkg/msg/AutonomousNavigationExampleAgentState.msg"
  "examples_pkg/msg/AutonomousNavigationExampleAgentReset.msg"
  "examples_pkg/srv/AutonomousNavigationExampleEnvironmentAction.srv"
  "examples_pkg/srv/AutonomousNavigationExampleEnvironmentState.srv"
  "examples_pkg/srv/AutonomousNavigationExampleEnvironmentReset.srv"

  #"examples_pkg/msg/EscoPilotExampleAgentAction.msg"
  #"examples_pkg/msg/EscoPilotExampleAgentState.msg"
  #"examples_pkg/srv/EscoPilotExampleEnvironmentStep.srv"

  "testing_pkg/msg/PoseSensorAndActuatorTestAgentAction.msg"
  "testing_pkg/msg/PoseSensorAndActuatorTestAgentState.msg"
  "testing_pkg/srv/PoseSensorAndActuatorTestEnvironmentAction.srv"
  "testing_pkg/srv/PoseSensorAndActuatorTestEnvironmentState.srv"
  "testing_pkg/srv/PoseSensorAndActuatorTestEnvironmentReset.srv"

  "testing_pkg/msg/TwistSensorAndActuatorTestAgentAction.msg"
  "testing_pkg/msg/TwistSensorAndActuatorTestAgentState.msg"
  "testing_pkg/srv/TwistSensorAndActuatorTestEnvironmentAction.srv"
  "testing_pkg/srv/TwistSensorAndActuatorTestEnvironmentState.srv"
  "testing_pkg/srv/TwistSensorAndActuatorTestEnvironmentReset.srv"

  "testing_pkg/msg/LidarSensorTestAgentAction.msg"
  "testing_pkg/msg/LidarSensorTestAgentState.msg"
  "testing_pkg/srv/LidarSensorTestEnvironmentAction.srv"
  "testing_pkg/srv/LidarSensorTestEnvironmentState.srv"
  "testing_pkg/srv/LidarSensorTestEnvironmentReset.srv"

  "testing_pkg/msg/TriggerSensorTestAgentAction.msg"
  "testing_pkg/msg/TriggerSensorTestAgentState.msg"
  "testing_pkg/srv/TriggerSensorTestEnvironmentAction.srv"
  "testing_pkg/srv/TriggerSensorTestEnvironmentState.srv"
  "testing_pkg/srv/TriggerSensorTestEnvironmentReset.srv"

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
