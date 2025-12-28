# MuJoCo MPC 汽车仪表盘项目

## 项目信息
- **学号**: 232011183
- **姓名**: 徐佳炜
- **班级**: 计科2305班
- **完成日期**: 2025年12月27日

## 项目概述

本项目基于 MuJoCo MPC 物理引擎实现了汽车仪表盘的实时可视化功能，核心是将仿真场景中车辆的物理状态数据，通过 OpenGL 渲染为 2D HUD 仪表盘。项目完成了速度表速 / 高温警告提示及仪表盘 UI 动画效果；数据层面实现了从 MuJoCo 原始仿真数据的提取、单位转换与平滑滤波，确保仪表盘指针无抖动，整体帧率稳定在 60FPS，满足实时交互需求。

## 环境要求
- 操作系统: Ubuntu 22.04
- 编译器: gcc 11.3.0
- CMake: 3.22.1


## 编译和运行

### 编译步骤
\`\`\`bash
cd mujoco_mpc
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build . -j4
\`\`\`

### 运行
\`\`\`bash
./bin/mjpc --task SimpleCar
\`\`\`

## 功能说明

### 已实现功能
- [x] 速度表
- [x] 转速表
- [x] 数字显示（油量、温度）
- [ ] 小地图（未实现）

### 进阶功能
- [x] UI动画效果
- [x] 警告提示

## 文件说明
- car_model.xml: 小车场景
- `simple_car.cc`: 小车函数
- `CMakeLists.txt`：构建函数

## 已知问题
- 车辆高速转向时，转速表数据会短暂波动

  

## 参考资料
- 开源代码：MuJoCo 官方车辆仿真示例（https://github.com/google-deepmind/mujoco_mpc/tree/main/mjpc/tasks/humanoid）
- MuJoCo中文入门教程
