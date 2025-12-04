cmake_minimum_required(VERSION 3.8)
project(demo_urdf)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# 安装URDF文件
install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

# 安装launch文件
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# 安装config文件
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# 安装meshes文件
install(DIRECTORY meshes/
  DESTINATION share/${PROJECT_NAME}/meshes
)

# 安装Python脚本
install(PROGRAMS
  scripts/publish_scene_to_moveit.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
