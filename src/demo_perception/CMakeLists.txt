cmake_minimum_required(VERSION 3.22)
project(mmk2_moveit_config)

find_package(ament_cmake REQUIRED)

ament_package()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

# 安装scripts目录中的Python脚本
install(PROGRAMS
  scripts/add_obstacle_node.py
  scripts/depth_to_pointcloud_node.py
  scripts/pointcloud_to_planning_scene.py
  DESTINATION lib/${PROJECT_NAME}
)
