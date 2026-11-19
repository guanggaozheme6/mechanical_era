# mechanical_era — 麦轮底盘 + 平面 2R 机械臂

| 项 | 内容 |
|---|---|
| MCU | STM32F103RCTx（256 KB Flash / 48 KB RAM，HSE 8 MHz × 9 = 72 MHz，LQFP64） |
| 骨架 | **自有分层**（`Core/` + `Driver/` + `Algorithm/` + `Device/` + `Task/`），不用 GSRL |
| 版本控制 | git · `origin 27EmbeddedGroupTranning/mechanical_era` · 分支 `main` |
| 产物 | `build/Debug/mechanical_era.elf` |

## 定位

一个 1 ms 主循环、一个 PS2 手柄、四路有刷电机（编码器速度闭环）、四路舵机。
控制代码 C++14，CubeMX 生成的 HAL 保持 C 不动。

**这个工程不用 GSRL**，是自己的分层，别把 GSRL 的写法硬套进来。

权威文档是 `README.md`（334 行）：分层职责表、接口、构建、仿真、v0.4 不兼容变更都在里面。
本文件只记「agent 干活时容易踩的」，细节以 README 为准。

## 构建与烧录

```powershell
cmake --preset Debug
cmake --build --preset Debug          # -> build\Debug\mechanical_era.elf
STM32CLI -w build\Debug\mechanical_era.elf -rst
```

- 工具链是 PATH 里的 `arm-none-eabi-`（GCC 15.3.1），编译选项 `-Wall -Wextra -Werror`。
- 当前基线：Debug `FLASH 46356 B (17.7%) / RAM 3104 B (6.3%)`，Release `FLASH 32648 B (12.5%)`，
  两者均 0 warning。**warning 数变了就是回归。**
- `flash.cfg` 已是本机配置（`interface/stlink.cfg` + `target/stm32f1x.cfg`），可以直接烧。
- **`build/` 是移动目录前留下的**：`CMakeCache.txt` 的 `CMAKE_HOME_DIRECTORY` 仍指向旧的
  `Workspace_Gmaster/mechanical_era`，直接 build 会报路径错。先删 `build/` 再 `cmake --preset Debug`。

## 目录结构

| 层 | 目录 | 禁止 |
|---|---|---|
| HAL | `Core/`、`Drivers/` | 只动 `USER CODE` 段 |
| Driver | `Driver/{inc,src}` | 不做闭环、不认业务 |
| Algorithm | `Algorithm/{inc,src}` | 不碰硬件、无全局状态、不读时钟 |
| Device | `Device/{inc,src}` | 不写寄存器时序 |
| Task | `Task/{inc,src}` | 不解析协议 |

头文件用裸文件名（`#include "arm_pose.hpp"`），CMake 按层加 `-I`。

其它文件：`mechanical_era.ioc`（CubeMX 工程）、`Debug.md`（待办与坑，改之前先翻）、
`sim/index.html`（二维仿真，双击即开，无后端）、`model/`、`Include/`、`ps2解码通讯手册V1.3.pdf`。

## 约定与纪律

- `v0.4` 是接口级精简：没有模式切换、没有 8 条预设姿态、机械臂改右摇杆。照旧版写法改会
  直接不成立，迁移前读 README §9。
- 改之前先翻 `Debug.md` 的待办与坑清单。
- 层间依赖只能自上而下：Task → Device → Algorithm → Driver → HAL，反向引用一律不允许。
- 注释按 `../../COMMENT_RULES.md`（doxygen 风格，与 GSRL 工程一致，但代码结构不用 GSRL）。

## 参考资料

- `README.md` — 本工程的权威文档（分层、接口、构建、仿真、迁移）。
- `Debug.md` — 待办与已知坑。
- `sim/index.html` — 二维仿真，双击即开。
- `ps2解码通讯手册V1.3.pdf` — PS2 手柄协议。

## 注释规则

见伞目录根 `../../COMMENT_RULES.md`；本文件不重复注释正文。
