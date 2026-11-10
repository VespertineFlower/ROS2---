# ROS 2 TurtleBot3 迷宫导航系统

本项目是《机器人感知与控制综合实践》课程综合设计，基于 **ROS 2 Humble**、**Gazebo Classic 11** 和 **TurtleBot3 Burger** 搭建矩形迷宫仿真环境，实现从基础运动控制到“两遍法”自主竞速的完整流程。

## 功能概览

- 自定义 5×5 Gazebo 迷宫与 TurtleBot3 仿真模型
- 直线、转向、方形轨迹等开环控制
- 基于里程计的点到点 P 控制与多航点跟踪
- 激光雷达六扇区距离提取及反应式避障
- 带转向锁存、切角和脱困策略的左墙跟随
- Pure Pursuit 轨迹跟踪
- 占据栅格建图、A* 路径规划与路径平滑
- 前馈 + PID / Pure Pursuit 两种竞速跟踪方式
- 控制器、规划器和迷宫解析器的自动化测试

## 运行环境

| 组件 | 版本 |
| --- | --- |
| Ubuntu | 22.04 |
| ROS 2 | Humble |
| Gazebo | Classic 11 |
| Python | 3.10 |
| 机器人 | TurtleBot3 Burger |

安装 ROS 依赖：

```bash
sudo apt update
sudo apt install ros-humble-desktop \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-turtlebot3 \
  ros-humble-turtlebot3-gazebo \
  python3-colcon-common-extensions \
  python3-pytest
```

## 项目结构

```text
.
├── maze_ws/                         ROS 2 工作空间
│   └── src/
│       ├── maze_description/        URDF、迷宫 world 和 RViz 配置
│       ├── maze_bringup/            各实验的 launch 文件
│       └── maze_control/            感知、控制、规划和测试代码
└── tools/
    ├── generate_maze.py             随机迷宫生成器
    ├── simulate_maze.py             无 Gazebo 的二维快速仿真器
    └── stop_all.sh                  清理项目相关仿真进程
```

## 编译

在项目根目录执行：

```bash
cd maze_ws
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
colcon build --symlink-install
source install/setup.bash
```

每次打开新终端后，需要重新加载环境：

```bash
source /opt/ros/humble/setup.bash
source <项目路径>/maze_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger
```

## 快速开始

### 启动完整迷宫任务

```bash
ros2 launch maze_bringup maze_run.launch.py
```

该命令会启动 Gazebo、TurtleBot3、RViz2、激光数据处理、轨迹记录和左墙跟随节点。机器人从入口出发，并在检测到离开北侧出口后停止。

无图形界面运行：

```bash
ros2 launch maze_bringup maze_run.launch.py gui:=false rviz:=false
```

调整主要参数：

```bash
ros2 launch maze_bringup maze_run.launch.py \
  speed:=0.22 target_left:=0.34 corner_speed:=0.25
```

### 其他实验

```bash
# 仅启动迷宫环境
ros2 launch maze_bringup maze_world.launch.py

# 开环方形轨迹
ros2 launch maze_bringup open_loop.launch.py mode:=square square_side:=0.7

# 点到点控制
ros2 launch maze_bringup point_control.launch.py goal_x:=1.2 goal_y:=0.4

# 自动规划并跟踪航点
ros2 launch maze_bringup waypoint_control.launch.py

# 反应式避障
ros2 launch maze_bringup reactive_avoid.launch.py front_stop:=0.45 speed:=0.16

# Pure Pursuit 跟踪
ros2 launch maze_bringup pure_pursuit.launch.py
```

## 两遍法：探索与竞速

第一遍沿左墙探索并将占据栅格保存为 `~/maze_map.npz`：

```bash
ros2 launch maze_bringup race.launch.py mode:=explore
```

探索结束后关闭仿真，再启动第二遍。程序会读取地图，执行 A* 规划、路径平滑和轨迹跟踪：

```bash
# 前馈 + PID
ros2 launch maze_bringup race.launch.py mode:=race controller:=pid

# Pure Pursuit
ros2 launch maze_bringup race.launch.py \
  mode:=race controller:=pure_pursuit lookahead:=0.30
```

探索和竞速使用自定义地图时，两次运行必须传入相同的 `map_file`。

## 主要 ROS 2 话题

| 话题 | 消息类型 | 用途 |
| --- | --- | --- |
| `/scan` | `sensor_msgs/msg/LaserScan` | 激光雷达原始数据 |
| `/odom` | `nav_msgs/msg/Odometry` | 机器人里程计 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 速度控制指令 |
| `/scan/front` | `std_msgs/msg/Float32` | 前方最小障碍距离 |
| `/scan/left` | `std_msgs/msg/Float32` | 左侧最小障碍距离 |
| `/scan/right` | `std_msgs/msg/Float32` | 右侧最小障碍距离 |
| `/actual_path` | `nav_msgs/msg/Path` | 实际运动轨迹 |
| `/reference_path` | `nav_msgs/msg/Path` | 规划参考路径 |
| `/map` | `nav_msgs/msg/OccupancyGrid` | 探索生成的栅格地图 |
| `/maze_done` | `std_msgs/msg/Bool` | 迷宫任务完成标志 |

## 测试

```bash
cd maze_ws
source /opt/ros/humble/setup.bash
colcon test --packages-select maze_control
colcon test-result --verbose
```

也可以直接运行 Python 单元测试：

```bash
cd maze_ws/src/maze_control
python3 -m pytest test/ -q
```

## 辅助工具

在项目根目录执行：

```bash
# 生成新的可解迷宫，并打印 ASCII 地图和最短路线
python3 tools/generate_maze.py \
  --output maze_ws/src/maze_description/worlds/maze.world

# 不启动 Gazebo，快速验证左墙跟随控制器
python3 tools/simulate_maze.py

# 清理 Gazebo、RViz2 和残留控制节点
bash tools/stop_all.sh
```

## 常见问题

**找不到功能包或节点**

确认已经成功编译，并在当前终端执行：

```bash
source /opt/ros/humble/setup.bash
source maze_ws/install/setup.bash
```

**找不到 TurtleBot3 模型**

```bash
export TURTLEBOT3_MODEL=burger
ros2 pkg prefix turtlebot3_gazebo
```

**机器人运动异常**

确保只有一个节点在发布 `/cmd_vel`。结束上一轮实验后再启动下一项；必要时运行 `bash tools/stop_all.sh` 清理残留进程。

**竞速模式找不到地图**

先完整运行 `mode:=explore`，确认 `~/maze_map.npz` 已生成；使用自定义路径时，探索和竞速阶段的 `map_file` 必须一致。

## 项目说明

TurtleBot3 模型和 Gazebo 插件来自 [ROBOTIS TurtleBot3](https://github.com/ROBOTIS-GIT/turtlebot3) 与 [TurtleBot3 Simulations](https://github.com/ROBOTIS-GIT/turtlebot3_simulations)。项目代码采用 Apache-2.0 许可证声明，具体以各 ROS 2 功能包的 `package.xml` 为准。

更详细的实验参数和逐项演示命令见 [maze_ws/readme.md](maze_ws/readme.md)。
