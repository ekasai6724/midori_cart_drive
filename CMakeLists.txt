cmake_minimum_required(VERSION 3.5)
project(midori_cart_drive)

# Enable ROS build
add_definitions(-DROS)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 依存パッケージ
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(midori_cart_messages REQUIRED)
find_package(Qt5 COMPONENTS Core Widgets REQUIRED)

# Qtライブラリのビルド設定
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

# インクルードファイルディレクトリ
include_directories(
	${rclcpp_INCLUDE_DIRS}
	${Qt5Core_INCLUDE_DIRS}
	${Qt5Quick_INCLUDE_DIRS}
)

# ビルド設定関数定義
function(custom_executable target)
    add_executable(
        ${target}
        src/${target}.cpp
    )
    ament_target_dependencies(
        ${target}
        rclcpp
        rclcpp_action
        geometry_msgs
        sensor_msgs
        nav_msgs
        nav2_msgs
        std_msgs
        tf2
        tf2_ros
        midori_cart_messages
    )
    qt5_use_modules(
        ${target}
        Core
        SerialPort
    )
    target_link_libraries(
        ${target}
        Qt5::Core
        Qt5::Widgets
        Qt5::SerialPort
    )
    install(
        TARGETS ${target}
        DESTINATION lib/${PROJECT_NAME}
    )
    install(
        DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}/
    )
    install(
        DIRECTORY urdf
        DESTINATION share/${PROJECT_NAME}/
    )
endfunction()

custom_executable(midori_cart_drive)
custom_executable(svon_client)
custom_executable(navigation_client)

ament_package()
