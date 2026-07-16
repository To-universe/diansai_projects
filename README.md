# 电赛训练项目仓库

本仓库用于管理三人电赛训练过程中的代码、文档、硬件记录和实验结果。主要开发平台为 STM32，少量项目会使用 TI 平台。

仓库的核心目标是：

- 统一保存每一次训练项目
- 方便三人共享和同步代码
- 记录硬件连接、调试过程和实验结果
- 避免通过压缩包、聊天文件、最终版文件反复传递代码

## 目录结构

每一次训练项目单独放在一个文件夹中，推荐命名方式：

```text
年份_项目名
```

例如：

```text
2025_G/
```

推荐单个项目结构如下：

```text
项目名/
├── README.md
├── APP/
│   ├── Inc/
│   └── Src/
├── Core/
│   ├── Inc/
│   └── Src/
├── Drivers/
├── cmake/
├── build/
├── result/
├── CMakeLists.txt
├── CMakePresets.json
├── 项目名.ioc
├── startup_xxx.s
└── xxx_FLASH.ld
```

其中：

- `APP/`：自己编写的应用层代码，建议把主要业务代码放在这里
- `Core/`：STM32CubeMX 生成和维护的核心代码
- `Drivers/`：HAL、CMSIS 等驱动文件
- `cmake/`：CMake 工具链和 CubeMX 生成的 CMake 配置
- `build/`：编译输出目录，不提交到 Git
- `result/`：需要长期保留的实验结果，例如最终固件、测试截图、波形图、报告

## STM32 项目约定

STM32 项目使用：

- STM32CubeMX 生成工程
- CMake 编译
- VS Code 编辑代码

应该提交到 Git 的内容：

```text
*.ioc
CMakeLists.txt
CMakePresets.json
cmake/
Core/
Drivers/
APP/
startup_xxx.s
xxx_FLASH.ld
README.md
```

不应该提交到 Git 的内容：

```text
build/
*.elf
*.hex
*.bin
*.map
*.o
*.d
.settings/
```

如果某次实验需要保留可直接烧录的固件，请放到：

```text
项目名/result/firmware/
```

## TI 项目约定

TI 项目使用 Code Composer Studio 官方编译方式。

通常应该提交：

```text
.project
.cproject
.ccsproject
targetConfigs/
*.c
*.h
*.cmd
README.md
```

通常不提交：

```text
Debug/
Release/
.metadata/
*.out
*.obj
*.map
*.d
*.mk
```

## Git 协作流程

每次开始写代码前，先同步远程仓库：

```bash
git pull
```

查看当前改动：

```bash
git status
```

添加需要提交的文件。推荐明确指定文件或文件夹：

```bash
git add README.md
git add 2025_G
```

如果确认当前所有改动都应该提交，也可以使用：

```bash
git add .
```

提交改动：

```bash
git commit -m "添加 STM32G474 基础工程"
```

推送到 GitHub：

```bash
git push
```

推荐完整流程：

```bash
git pull
git status
git add 要提交的文件或文件夹
git status
git commit -m "说明这次完成了什么"
git push
```

## Commit 信息规范

Commit 信息使用中文，简短说明这次完成了什么。

推荐：

```text
添加 STM32G474 基础工程
完成 GPIO 初始化
添加 AD9851 驱动文件
修复 PWM 占空比计算错误
更新 2025_G 项目说明
```

不推荐：

```text
test
111
改了点东西
最终版
最终版2
```

## 协作注意事项

- 开始工作前先 `git pull`
- 完成一个小功能后及时 `commit`
- 不要多人同时大量修改同一个文件，尤其是 `main.c`
- 主要业务代码尽量放到 `APP/Inc` 和 `APP/Src`
- CubeMX 重新生成代码前，注意保护自己写在用户代码区之外的内容
- 提交前使用 `git status` 检查是否误加入了编译文件
- 不要提交压缩包、临时文件和无意义的最终版文件

## 当前项目

```text
2025_G/
```

这是当前仓库中的第一个 STM32 训练项目。后续建议在 `2025_G/README.md` 中继续补充该项目的具体内容，例如：

- 项目目标
- 使用芯片
- 外设模块
- 硬件连接
- 三人分工
- 编译和烧录方法
- 当前完成进度
