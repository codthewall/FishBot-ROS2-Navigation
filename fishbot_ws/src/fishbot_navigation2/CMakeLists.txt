cmake_minimum_required(VERSION 3.8)
project(fishbot_navigation2)

if(CMAKE_COMPILER_IS_GNUCXX OR MAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
# 添加 GICP 重定位包依赖（确保编译顺序）
find_package(gicp_relocalizer REQUIRED)

# 安装Python脚本为可执行文件
ament_python_install_package(scripts)

install(PROGRAMS
  scripts/auto_relocalization.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# 安装目录
install(DIRECTORY 
  launch
  config
  maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
