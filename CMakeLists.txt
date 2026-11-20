cmake_minimum_required(VERSION 3.22)
project(mechanical_era LANGUAGES C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()
enable_language(ASM)
add_executable(${PROJECT_NAME})
add_subdirectory(cmake/stm32cubemx)
target_sources(${PROJECT_NAME} PRIVATE
    # Task 层：业务编排 / 控制循环 / 模式状态机
    Task/src/main.cpp
    Task/src/mode_control.cpp
    Task/src/diagnostics.cpp
    # Device 层：协议解析 / 设备状态机 / 闭环接口
    Device/src/ps2_protocol.cpp
    Device/src/drive_control.cpp
    Device/src/arm_control.cpp
    Device/src/motor.cpp
    Device/src/arm_joint.cpp
    Device/src/arm_pose.cpp
    # Algorithm 层：纯计算（IK/FK、PID、摇杆限位）
    Algorithm/src/arm_kinematics.cpp
    Algorithm/src/arm_limit.cpp
    Algorithm/src/pid.cpp
    # Driver 层：寄存器 / PWM / SPI 时序
    Driver/src/ps2.cpp
    Driver/src/outputs.cpp
    Driver/src/encoder.cpp
)
target_include_directories(${PROJECT_NAME} PRIVATE
    Include
    Task/inc
    Device/inc
    Algorithm/inc
    Driver/inc
)
target_compile_options(${PROJECT_NAME} PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-Wall;-Wextra;-Werror>)
# 某些工具链错误地将 ob 加入 C 隐式链接库。
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)
target_link_libraries(${PROJECT_NAME} stm32cubemx)
