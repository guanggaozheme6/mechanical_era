/**
 ******************************************************************************
 * @file           : app.h
 * @brief          : Core/Src/main.c 与 C++ 应用层之间的唯一桥接声明
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 应用主任务
 * @details 由 Core/Src/main.c 在 HAL 初始化完成后调用一次，内部是永不返回的主循环。
 * @note 定义在 Task/src/main.cpp。该定义必须带 extern "C"，否则 C++ 会做名字修饰，
 *       C 侧按裸符号找不到，链接报 undefined reference。
 * @note 本头文件只暴露这一个函数：不允许出现 Ps2Frame 等 C++ 类型，否则 C 侧编不过。
 */
void main_task(void);

#ifdef __cplusplus
}
#endif
