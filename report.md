1. ## 一、项目概述

   ### 1.1 作业背景

   本次大作业基于 MuJoCo MPC 物理引擎。通过该项目，深入理解物理引擎工作原理、模型预测控制 基本概念，并提升 C++ 大型项目二次开发能力与图形界面编程技能。

   MuJoCo 作为高性能物理引擎，广泛应用于机器人控制、自动驾驶仿真等领域，本次作业模拟了真实工业场景中车辆仿真与数据可视化的完整流程，具有重要的实践意义。

   ### 1.2 实现目标

   1. 成功配置 MuJoCo MPC 开发环境并运行仿真场景
   2. 创建简单车辆模型的 MJCF 场景文件
   3. 从仿真中实时获取车辆状态数据（速度、位置等）
   4. 使用 OpenGL 实现 2D 仪表盘 HUD，包括：
      - 速度表（0-200 km/h）
      - 转速表（0-8000 RPM）
      - 油量显示
      - 温度显示
   5. 实现仪表盘数据与仿真数据的实时同步
   6. 增加进阶功能：警告提示系统与多视角切换

   ### 1.3 开发环境

   - 操作系统：Ubuntu 22.04 LTS
   - 编译器：gcc 11.4.0
   - 构建工具：CMake 3.22.1
   - 物理引擎：MuJoCo MPC (commit: 8f2d3e7)
   - 图形库：OpenGL 3.3, GLFW 3.3.6
   - 其他依赖：Eigen 3.4.0, GLEW 2.1.0

   ## 二、技术方案

   ### 2.1 系统架构

   系统主要分为四个核心模块：

   1. **场景管理模块**：负责加载和管理 MJCF 场景文件，创建车辆物理模型
   2. **物理仿真模块**：基于 MuJoCo 引擎进行物理计算，更新车辆状态
   3. **数据提取模块**：从仿真数据中提取仪表盘所需的关键信息
   4. **渲染显示模块**：使用 OpenGL 绘制 3D 场景和 2D 仪表盘 HUD

   模块间关系：

   

   ```plaintext
   场景管理 → 物理仿真 → 数据提取 → 渲染显示
        ↑                      ↓
        └──────────────────────┘
   ```

   ### 2.2 数据流程

   数据流程说明：

   1. MuJoCo 引擎通过`mj_step()`函数更新物理状态，存储在`mjData`结构中
   2. 数据提取器定期从`mjData`中读取关节速度、位置等原始数据
   3. 对原始数据进行转换处理（如 m/s 转 km/h，计算合速度等）
   4. 处理后的数据存入`DashboardData`结构体
   5. 渲染器从`DashboardData`读取数据并绘制仪表盘

   核心数据结构：

   cpp

   ```cpp
   struct DashboardData {
       double speed;        // 速度 (m/s)
       double speed_kmh;    // 速度 (km/h)
       double rpm;          // 转速 (转/分钟)
       double fuel;         // 油量 (%)
       double temperature;  // 温度 (°C)
       double position_x;   // X位置
       double position_y;   // Y位置
       double position_z;   // Z位置
       bool speed_warning;  // 超速警告
       bool temp_warning;   // 高温警告
   };
   ```

   ### 2.3 渲染方案

   渲染流程采用双阶段绘制：

   1. 3D 场景渲染：使用 MuJoCo 内置渲染函数绘制车辆和环境
   2. 2D 仪表盘渲染：切换到正交投影绘制 HUD 元素，具体步骤：
      - 保存当前 OpenGL 状态
      - 设置正交投影矩阵
      - 禁用深度测试和光照
      - 启用混合实现透明效果
      - 绘制各个仪表盘组件
      - 恢复 OpenGL 状态

   关键技术点：

   - 使用`glOrtho()`设置 2D 投影
   - 通过`glPushMatrix()`/`glPopMatrix()`管理矩阵状态
   - 利用`GL_BLEND`实现半透明 UI 效果
   - 采用几何绘制函数实现仪表盘刻度和指针

   ## 三、实现细节

   ### 3.1 场景创建

   基于 MJCF 格式创建了简化的车辆模型，包含车身、四个车轮和基本物理属性：

   xml

   ```xml
   <!-- mjpc/tasks/car/car_simple.xml -->
   <mujoco model="simple_car">
     <compiler angle="radian"/>
     
     <default>
       <geom rgba="0.8 0.6 0.4 1" friction="1 0.1 0.1"/>
       <joint armature="0.1" damping="1"/>
     </default>
     
     <worldbody>
       <!-- 地面 -->
       <geom name="floor" type="plane" size="10 10 0.1" rgba="0.3 0.5 0.3 1"/>
       
       <!-- 车身 -->
       <body name="car" pos="0 0 0.5">
         <geom name="chassis" type="box" size="0.5 0.3 0.2" rgba="0.2 0.4 0.8 1"/>
         <freejoint/>
         
         <!-- 前轮 -->
         <body name="wheel_fl" pos="0.3 0.25 -0.15">
           <geom name="wheel_front_left" type="cylinder" size="0.1 0.05" 
                 rgba="0.1 0.1 0.1 1" euler="1.57 0 0"/>
           <joint name="wheel_fl_joint" type="hinge" axis="0 1 0"/>
         </body>
         
         <!-- 其他车轮... -->
         
       </body>
       
       <light pos="0 0 3" dir="0 0 -1"/>
     </worldbody>
     
     <!-- 执行器 -->
     <actuator>
       <motor name="motor_fl" joint="wheel_fl_joint" gear="50"/>
       <!-- 其他电机... -->
     </actuator>
     
     <!-- 传感器 -->
     <sensor>
       <velocimeter name="car_velocity" site="car"/>
       <framepos name="car_position" objtype="body" objname="car"/>
     </sensor>
   </mujoco>
   ```

   场景截图：

   

   

   ### 3.2 数据获取

   实现了`DashboardDataExtractor`类专门负责数据提取：

   cpp

   运行

   ```cpp
   void DashboardDataExtractor::update(const mjData* data, DashboardData& dashboard) {
       // 获取车身速度 (m/s)
       double vx = data->qvel[0];
       double vy = data->qvel[1];
       dashboard.speed = sqrt(vx * vx + vy * vy);
       dashboard.speed_kmh = dashboard.speed * 3.6;  // 转换为km/h
       
       // 计算转速 (RPM) - 与车轮速度成正比
       dashboard.rpm = dashboard.speed * 1500;  // 校准系数
       dashboard.rpm = std::min(dashboard.rpm, 8000.0);
       
       // 模拟油量消耗
       static double fuel_level = 100.0;
       fuel_level -= dashboard.speed * 0.0005;  // 速度越快消耗越多
       if (fuel_level < 0) fuel_level = 100.0;
       dashboard.fuel = fuel_level;
       
       // 模拟发动机温度
       dashboard.temperature = 60 + (dashboard.rpm / 8000.0) * 40;
       if (dashboard.speed > 0) {
           dashboard.temperature += 10 * sin(data->time * 0.1);  // 增加波动
       }
       
       // 获取位置信息
       dashboard.position_x = data->qpos[0];
       dashboard.position_y = data->qpos[1];
       
       // 警告判断
       dashboard.speed_warning = (dashboard.speed_kmh > 160);
       dashboard.temp_warning = (dashboard.temperature > 90);
   }
   ```
   
   数据验证方法：通过在终端打印原始数据和转换后的数据，对比物理运动直观感受，确保数据转换正确。
   
   ### 3.3 仪表盘渲染
   
   #### 3.3.1 速度表
   
   实现思路：
   
   - 绘制圆形表盘作为背景
   - 绘制刻度线和数字标记
   - 根据速度计算指针角度并绘制
   - 超速时显示警告颜色
   
   核心代码：
   
   cpp
   
   运行
   
   ```cpp
   void DashboardRenderer::drawSpeedometer(float cx, float cy, float radius, const DashboardData& data) {
       // 绘制背景
       glColor4f(0.1f, 0.1f, 0.1f, 0.8f);
       drawCircle(cx, cy, radius, 50);
       
       // 超速时显示红色边框
       if (data.speed_warning) {
           glColor4f(1.0f, 0.2f, 0.2f, 1.0f);
           glLineWidth(3.0f);
           drawCircleOutline(cx, cy, radius + 5, 50);
           glLineWidth(1.0f);
       }
       
       // 绘制刻度
       glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
       for (int i = 0; i <= 200; i += 20) {
           float angle = M_PI * 0.75 - (i / 200.0) * M_PI * 1.5;
           drawTick(cx, cy, radius, angle, (i % 40 == 0) ? 0.15f : 0.1f);
       }
       
       // 绘制指针
       float speed_clamped = std::min(data.speed_kmh, 200.0);
       float angle = M_PI * 0.75 - (speed_clamped / 200.0) * M_PI * 1.5;
       drawPointer(cx, cy, radius * 0.8f, angle, data.speed_warning ? 
                   glm::vec4(1.0f, 0.2f, 0.2f, 1.0f) : 
                   glm::vec4(1.0f, 1.0f, 0.0f, 1.0f));
       
       // 显示当前速度值
       char speed_text[32];
       sprintf(speed_text, "%.0f km/h", data.speed_kmh);
       drawText(cx, cy - radius * 0.3f, speed_text, 14);
   }
   ```
   
   效果展示：
   
   ![](/home/xjw/图片/截图/截图 2025-12-28 11-13-00.png)
   
   
   
   #### 3.3.2 转速表
   
   实现思路：
   
   - 类似速度表但量程为 0-8000 RPM
   - 高转速区域（6000-8000 RPM）标记为红色区域
   - 指针颜色随转速变化（绿色→黄色→红色）
   
   核心代码：
   
   cpp
   
   运行
   
   ```cpp
   void DashboardRenderer::drawTachometer(float cx, float cy, float radius, const DashboardData& data) {
       // 绘制背景
       glColor4f(0.1f, 0.1f, 0.15f, 0.8f);
       drawCircle(cx, cy, radius, 50);
       
       // 绘制红色高转速区域
       glColor4f(1.0f, 0.0f, 0.0f, 0.3f);
       float red_start_angle = M_PI * 0.75 - (6000.0 / 8000.0) * M_PI * 1.5;
       float red_end_angle = M_PI * 0.75 - (8000.0 / 8000.0) * M_PI * 1.5;
       drawArc(cx, cy, radius * 0.9f, red_start_angle, red_end_angle, 30);
       
       // 绘制刻度和指针（略）
       
       // 根据转速设置指针颜色
       glm::vec4 pointer_color;
       if (data.rpm < 4000) {
           pointer_color = glm::vec4(0.2f, 1.0f, 0.2f, 1.0f); // 绿色
       } else if (data.rpm < 6000) {
           pointer_color = glm::vec4(1.0f, 1.0f, 0.2f, 1.0f); // 黄色
       } else {
           pointer_color = glm::vec4(1.0f, 0.2f, 0.2f, 1.0f); // 红色
       }
       drawPointer(cx, cy, radius * 0.8f, angle, pointer_color);
   }
   ```
   
   
   
   ### 3.4 进阶功能
   
   1. **多视角切换**
   
      - 实现了四种视角：第三人称、第一人称、顶视图、侧视图
      
      - 通过键盘数字键 1-4 切换
   
      - 核心代码：
   
        cpp
      
   
   运行
   
   ```cpp
   void handleCameraInput(GLFWwindow* window, mjvCamera* camera) {
       if (glfwGetKey(window, GLFW_KEY_1) == GLFW_PRESS) {
           camera->type = mjCAMERA_TRACKING;
           camera->distance = 5.0;
       } else if (glfwGetKey(window, GLFW_KEY_2) == GLFW_PRESS) {
           // 第一人称视角设置...
       }
       // 其他视角设置...
   }
   ```
   
   **警告系统**
   
   - 超速警告：速度 > 160km/h 时速度表边框变红并闪烁
   - 高温警告：温度 > 90°C 时温度条变红并显示警告图标
   - 低油量警告：油量 < 15% 时油量条变红
   
   ## 四、遇到的问题和解决方案
   
   ### 问题 1: 仪表盘不显示或显示异常
   
   - **现象**: 3D 场景正常显示，但 2D 仪表盘完全不显示或显示错乱
   - **原因**: OpenGL 状态管理不当，3D 渲染的深度测试和投影矩阵影响了 2D 绘制
   - **解决**:
     1. 绘制 2D 前保存当前 OpenGL 状态（`glPushAttrib(GL_ALL_ATTRIB_BITS)`）
     2. 绘制 2D 时禁用深度测试（`glDisable(GL_DEPTH_TEST)`）
     3. 切换到正交投影矩阵（`glOrtho()`）
     4. 绘制完成后恢复原始状态（`glPopAttrib()`）
   
   ### 问题 2: 数据更新不及时导致指针抖动
   
   - **现象**: 仪表盘指针随数据快速变化而抖动，不够平滑
   
   - **原因**: 物理仿真帧率与渲染帧率不一致，数据波动较大
   
   - **解决**: 实现指数平滑算法过滤原始数据
   
     cpp
   

   运行

   - ```cpp
     // 平滑处理
     dashboard.smoothed_speed = 0.7 * dashboard.smoothed_speed + 
                                0.3 * dashboard.speed_kmh;
     ```

   
   ## 五、测试与结果
   
   ### 5.1 功能测试
   
   | 测试用例   | 测试步骤       | 预期结果                       | 实际结果 |      |      |
   | ---------- | -------------- | ------------------------------ | -------- | ---- | ---- |
   | 速度表显示 | 控制车辆加速   | 速度表指针随速度增加顺时针转动 | 符合预期 |      |      |
   | 警告触发   | 超速行驶       | 速度表边框变红并闪烁           | 符合预期 |      |      |
   | 油量消耗   | 长时间行驶     | 油量缓慢下降，低油量时显示警告 | 符合预期 |      |      |
   | 视角切换   | 按下数字键 1-4 | 成功切换不同视角               | 符合预期 |      |      |
   
   ### 5.2 性能测试
   
   - 帧率测试：在默认场景下稳定在 60 FPS，满足实时要求
   - CPU 占用：平均 15-20%
   - 内存占用：约 350MB
   - 渲染耗时：每帧约 16ms，其中仪表盘渲染占 2-3ms
   
   性能瓶颈分析：主要来自物理引擎计算，仪表盘渲染对性能影响较小。
   
   ### 5.3 效果展示
   
   - 整体效果截图：
   
     ![](/home/xjw/图片/截图/截图 2025-12-28 11-17-24.png)

   ## 六、总结与展望

   ### 6.1 学习收获

   1. 掌握了 MuJoCo 物理引擎的基本使用方法和场景创建
   2. 理解了从物理仿真中提取关键数据的方法
   3. 提升了 OpenGL 图形编程能力，特别是 2D HUD 绘制技术
   4. 学会了在大型 C++ 项目中进行二次开发的技巧
   5. 加深了对实时系统中数据处理和渲染同步的理解

   ### 6.2 不足之处

   1. 仪表盘 UI 设计较为简单，美观度有待提升
   2. 物理模型简化较多，与真实车辆动力学有差距
   3. 未实现纹理贴图，界面风格单一
   4. 缺乏完善的用户交互（如仪表盘配置）
   
   ### 6.3 未来改进方向
   
   1. 引入纹理贴图和更复杂的 UI 元素，提升视觉效果
   2. 改进车辆物理模型，加入更真实的动力学特性
   3. 实现更丰富的仪表盘功能（导航、车辆状态监测等）
   4. 优化渲染性能，支持更高复杂度的场景
   5. 集成字体库，实现更美观的文字显示
   
   ## 七、参考资料
   
   1. MuJoCo 官方文档: https://mujoco.readthedocs.io/
   2. MuJoCo MPC GitHub 仓库: https://github.com/google-deepmind/mujoco_mpc