cmake_minimum_required(VERSION 3.8)
project(ros_socket_can_driver)

# 必要なパッケージを探す
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(ament_index_cpp REQUIRED)

# カスタムメッセージの生成
set(msg_files
  "msg/CANFrame.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs
)

# インクルードディレクトリをエクスポート
ament_export_include_directories(include)
include_directories(include)

# YAML-CPPのインクルードディレクトリを設定
include_directories(${YAML_CPP_INCLUDE_DIR})

# エクスポートする依存パッケージの設定
ament_export_dependencies(rclcpp std_msgs yaml-cpp rosidl_default_runtime rosidl_default_generators ament_index_cpp)

# ビルド対象を指定
add_executable(socket_can_driver src/socket_can_driver.cpp src/socket_can_driver_node.cpp)

# 必要な依存関係を指定
ament_target_dependencies(socket_can_driver rclcpp std_msgs yaml-cpp rosidl_default_runtime rosidl_default_generators ament_index_cpp)

# ターゲットのリンク
target_link_libraries(socket_can_driver ${YAML_CPP_LIBRARIES})

# typesupportターゲットのリンク
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(socket_can_driver "${cpp_typesupport_target}")

# インストール設定
install(TARGETS 
  socket_can_driver
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.yaml"
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.launch.py"
)

ament_package()
