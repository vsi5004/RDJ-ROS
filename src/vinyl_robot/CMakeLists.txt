cmake_minimum_required(VERSION 3.8)
project(vinyl_robot)

find_package(ament_cmake REQUIRED)

# ── bus.yml path substitution ──────────────────────────────────────────────────
# @BUS_CONFIG_PATH@ in bus.yml.in is replaced with the installed config directory
# path so ros2_canopen can locate EDS files at runtime without any extra setup.
set(BUS_CONFIG_PATH "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/config")
configure_file(
  ../../config/bus.yml.in
  ${CMAKE_CURRENT_BINARY_DIR}/bus.yml
  @ONLY
)

# ── Install launch files ───────────────────────────────────────────────────────
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

# ── Install config files ───────────────────────────────────────────────────────
# Install the generated bus.yml (with path substituted) rather than the source .in file.
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/bus.yml
  DESTINATION share/${PROJECT_NAME}/config)

install(FILES
  ../../config/robot_params.yaml
  ../../config/mock_ros2_control.yaml
  ../../config/stepper_node.eds
  ../../config/a_axis_node.eds
  ../../config/servo_node.eds
  DESTINATION share/${PROJECT_NAME}/config)

ament_package()
