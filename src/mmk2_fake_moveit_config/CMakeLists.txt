cmake_minimum_required(VERSION 3.22)
project(mmk2_fake_moveit_config)

find_package(ament_cmake REQUIRED)

ament_package()

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/fake_joint_driver.py
  DESTINATION lib/${PROJECT_NAME}
)
