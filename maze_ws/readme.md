# 地老鼠走迷宫系统

本项目为《机器人感知与控制综合实践》课程综合设计，基于 ROS 2 Humble、Gazebo Classic 11 和 TurtleBot3 Burger 完成矩形迷宫仿真。系统包含机器人及环境加载、开环运动、里程计闭环控制、航点跟踪、激光雷达分区、反应式避障、左墙跟随，以及“探索建图—A* 规划—轨迹跟踪”的两遍法拓展。

- 课程代码：`(2026-2027-1)-S0610350-02`

## 1 项目功能

### 1.1 基础任务

- 在 Gazebo 中加载自定义矩形墙体迷宫和 TurtleBot3 Burger。
- 使用 Python launch 一键启动 Gazebo、机器人模型、RViz2 和算法节点。
- 实现直线、原地转向和方形轨迹等开环运动。
- 基于 `/odom` 实现 P 控制点到点运动。
- 实现给定航点序列或自动规划航点序列的跟踪。
- 解析 `/scan`，提取前方、左侧、右侧等六个方向的最小障碍距离。
- 实现前方障碍检测和反应式避障。
- 实现带转向锁存、门口切角、连续 S 弯和倒车脱困的左墙跟随。

### 1.2 拓展任务

- Pure Pursuit 纯追踪轨迹跟踪。
- 第一遍左墙探索并保存占据栅格地图。
- 第二遍在地图上执行 A* 路径规划、路径平滑和速度剖面计算。
- 支持前馈 + PID 或 Pure Pursuit 两种竞速跟踪器。

## 2 运行环境

| 项目 | 版本或说明 |
| --- | --- |
| 操作系统 | Ubuntu 22.04 |
| ROS 2 | Humble |
| 仿真器 | Gazebo Classic 11 |
| 机器人 | TurtleBot3 Burger |
| Python | 3.10 |
| 构建工具 | colcon |

安装依赖：

```bash
sudo apt update
sudo apt install ros-humble-desktop \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-turtlebot3 \
  ros-humble-turtlebot3-gazebo
```

加载 ROS 环境并指定机器人型号：

```bash
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 pkg prefix turtlebot3_gazebo
```

## 3 工作空间结构

```text
maze_ws/
├── readme.md
└── src/
    ├── maze_description/            机器人、世界与 RViz 资源
    │   ├── urdf/maze_bot.urdf       第一版自建模型，保留作对照
    │   ├── worlds/empty.world       基础运动实验使用的空世界
    │   ├── worlds/maze.world        当前 5×5 迷宫世界
    │   └── rviz/maze.rviz           RViz2 显示配置
    ├── maze_bringup/                Python launch 启动编排
    │   └── launch/
    │       ├── maze_common.py       Gazebo、模型和 RViz 公共启动逻辑
    │       ├── maze_world.launch.py
    │       ├── maze_run.launch.py
    │       ├── open_loop.launch.py
    │       ├── point_control.launch.py
    │       ├── waypoint_control.launch.py
    │       ├── reactive_avoid.launch.py
    │       ├── pure_pursuit.launch.py
    │       └── race.launch.py
    └── maze_control/                ROS 2 控制与规划节点
        ├── maze_control/
        │   ├── control_law.py       P 控制、纯追踪和左墙状态机核心算法
        │   ├── scan_processor.py    LaserScan 六扇区距离提取
        │   ├── reactive_avoid.py    反应式避障
        │   ├── wall_follower.py     左墙跟随节点
        │   ├── map_builder.py       占据栅格建图
        │   ├── race_planner.py      A*、路径平滑和速度剖面
        │   ├── pid_tracker.py       前馈 + PID 跟踪器
        │   ├── path_racer.py        第二遍竞速节点
        │   ├── world_maze.py        world 解析和自动路线生成
        │   └── 其他运动控制节点
        └── test/                    控制、规划和状态机单元测试
```

`build/`、`install/` 和 `log/` 由 colcon 自动生成，不需要提交到代码仓库或课程代码压缩包。

## 4 编译

```bash
cd ~/Documents/maze_ws
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
colcon build --symlink-install
source install/setup.bash
```

每次打开新终端都需要重新执行：

```bash
source /opt/ros/humble/setup.bash
source ~/Documents/maze_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger
```

## 5 启动与演示

所有 launch 默认同时打开 Gazebo 图形界面和 RViz2。无图形界面运行时可追加 `gui:=false rviz:=false`。

### 5.1 查看迷宫和机器人模型

```bash
ros2 launch maze_bringup maze_world.launch.py
```

仅在 RViz2 中检查机器人模型：

```bash
ros2 launch maze_bringup display.launch.py
```

### 5.2 开环运动

```bash
# 直线行驶
ros2 launch maze_bringup open_loop.launch.py \
  mode:=straight distance:=1.0 linear_speed:=0.15

# 原地左转 90°
ros2 launch maze_bringup open_loop.launch.py \
  mode:=turn_left angle:=1.5708 angular_speed:=0.8

# 方形轨迹
ros2 launch maze_bringup open_loop.launch.py mode:=square square_side:=0.7
```

`mode` 可选 `straight`、`turn_left`、`turn_right` 或 `square`。

### 5.3 点到点 P 控制

```bash
ros2 launch maze_bringup point_control.launch.py goal_x:=1.20 goal_y:=0.40
```

不指定坐标时，节点会读取 `maze.world`，从起点选择一段可直达的目标。点到点控制不包含主动避障，因此显式目标必须位于无墙阻挡的直线上。

### 5.4 航点序列跟踪

指定航点：

```bash
ros2 launch maze_bringup waypoint_control.launch.py \
  waypoints:="0.4,0.4;1.2,0.4;1.2,1.2"
```

不传 `waypoints` 时，节点会根据 `maze.world` 自动生成通往出口的航点序列：

```bash
ros2 launch maze_bringup waypoint_control.launch.py
```

### 5.5 反应式避障

```bash
ros2 launch maze_bringup reactive_avoid.launch.py \
  front_stop:=0.45 speed:=0.16
```

### 5.6 左墙跟随核心任务

```bash
ros2 launch maze_bringup maze_run.launch.py
```

该命令启动迷宫世界、TurtleBot3、RViz2、雷达分区节点、轨迹记录节点和左墙跟随节点。机器人从 `(0.40, 0.40)` 出发，依靠激光雷达与里程计沿墙行驶并从北侧出口离开。

常用参数：

```bash
ros2 launch maze_bringup maze_run.launch.py \
  speed:=0.22 target_left:=0.34 corner_speed:=0.25
```

关闭门口切角、使用原地转向：

```bash
ros2 launch maze_bringup maze_run.launch.py corner_enabled:=0.0
```

### 5.7 Pure Pursuit 纯追踪

自动读取迷宫并规划参考路线：

```bash
ros2 launch maze_bringup pure_pursuit.launch.py
```

使用自定义折线路径：

```bash
ros2 launch maze_bringup pure_pursuit.launch.py \
  path:="0.4,0.4;1.2,0.4;1.2,1.2" \
  lookahead:=0.32 max_linear:=0.20
```

### 5.8 两遍法

第一遍使用左墙跟随探索，并将地图保存为 `~/maze_map.npz`：

```bash
ros2 launch maze_bringup race.launch.py mode:=explore
```

第一遍结束并关闭节点后，确认地图文件存在：

```bash
ls -lh ~/maze_map.npz
```

第二遍读取地图，执行 A* 规划、路径平滑和竞速跟踪：

```bash
ros2 launch maze_bringup race.launch.py mode:=race controller:=pid
```

切换为 Pure Pursuit 跟踪器：

```bash
ros2 launch maze_bringup race.launch.py \
  mode:=race controller:=pure_pursuit lookahead:=0.30
```

如需使用其他地图文件，两遍必须传入相同的 `map_file`：

```bash
ros2 launch maze_bringup race.launch.py \
  mode:=explore map_file:=/tmp/maze_map.npz

ros2 launch maze_bringup race.launch.py \
  mode:=race map_file:=/tmp/maze_map.npz controller:=pid
```

## 6 常用话题

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/scan` | `sensor_msgs/msg/LaserScan` | 360° 激光雷达数据 |
| `/odom` | `nav_msgs/msg/Odometry` | 机器人里程计 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 线速度和角速度指令 |
| `/scan/front` | `std_msgs/msg/Float32` | 前方扇区最小距离 |
| `/scan/left` | `std_msgs/msg/Float32` | 左侧扇区最小距离 |
| `/scan/right` | `std_msgs/msg/Float32` | 右侧扇区最小距离 |
| `/actual_path` | `nav_msgs/msg/Path` | 里程计记录的实际轨迹 |
| `/reference_path` | `nav_msgs/msg/Path` | 规划或跟踪参考路径 |
| `/map` | `nav_msgs/msg/OccupancyGrid` | 第一遍生成的占据栅格地图 |
| `/maze_done` | `std_msgs/msg/Bool` | 左墙跟随完成标志 |

查看话题和节点：

```bash
ros2 node list
ros2 topic list
ros2 topic echo /scan/front
ros2 topic hz /scan
ros2 topic hz /odom
```

## 7 测试

```bash
cd ~/Documents/maze_ws/src/maze_control
source /opt/ros/humble/setup.bash
python3 -m pytest test/ -q
```

也可以运行 colcon 测试：

```bash
cd ~/Documents/maze_ws
source /opt/ros/humble/setup.bash
colcon test --packages-select maze_control
colcon test-result --verbose
```

## 8 常见问题

### 8.1 找不到 TurtleBot3 模型

```bash
ros2 pkg prefix turtlebot3_gazebo
export TURTLEBOT3_MODEL=burger
```

### 8.2 找不到功能包或节点

```bash
source /opt/ros/humble/setup.bash
source ~/Documents/maze_ws/install/setup.bash
```

若仍然失败，重新执行 `colcon build --symlink-install` 并加载 `install/setup.bash`。

### 8.3 机器人运动异常

关闭上一轮 Gazebo、RViz2 和 ROS 2 launch 后再启动下一项实验。多个控制节点同时发布 `/cmd_vel` 会造成运动异常。

### 8.4 第二遍找不到地图

先完整运行 `mode:=explore`，结束后检查 `~/maze_map.npz`。如果使用自定义 `map_file`，探索和竞速两次启动必须使用同一路径。

## 9 个人分工

本项目为单人组，由何泽羽独立完成：

- 环境与模型：迷宫 world、RViz 配置及 TurtleBot3 仿真加载。
- 启动编排：`maze_bringup` 中的 Python launch 文件。
- 运动控制：开环、点到点 P 控制、航点跟踪和 Pure Pursuit。
- 感知与决策：激光分区、反应式避障及左墙跟随状态机。
- 拓展功能：栅格建图、A* 规划、速度剖面和前馈 + PID 跟踪。
- 测试与文档：单元测试、仿真实验、综合设计报告和本 README。

## 10 外部资源与 AI 使用说明

TurtleBot3 模型和 Gazebo 插件来自 ROBOTIS 官方开源项目：

- <https://github.com/ROBOTIS-GIT/turtlebot3>
- <https://github.com/ROBOTIS-GIT/turtlebot3_simulations>
- <https://index.ros.org/p/turtlebot3_gazebo/>

本项目在节点骨架、launch 模板、单元测试和文档整理过程中使用了 AI 编程助手。所有控制逻辑、参数和实验结果均经过人工检查、单元测试与 Gazebo 仿真验证。
