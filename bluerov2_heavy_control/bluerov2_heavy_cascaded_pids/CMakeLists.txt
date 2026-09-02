cmake_minimum_required(VERSION 3.8)
project(bluerov2_heavy_cascaded_pids)

# find dependencies
find_package(ament_cmake REQUIRED)

ament_python_install_package(bluerov2_heavy_PID PACKAGE_DIR src/PID)

install(PROGRAMS scripts/position_control.py
                 scripts/position_control_bluerov2_heavy.py
                 scripts/velocity_control_bluerov2_heavy.py
                 scripts/velocity_control.py
                 scripts/acceleration_control.py
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*~"
  EXCLUDE)
ament_package()
