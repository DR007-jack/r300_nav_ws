## 快速概要 — 目的
本文件为 AI 编程 agent（例如 Copilot 型 agent）提供立即可用的、面向此仓库的上下文与指引。目标是让 agent 在本 ROS (catkin) 工作区中快速定位主要组件、常用构建/启动流程、以及可直接修改或运行的示例。

## 仓库“全景”和主要组件
- 工作区类型：ROS (catkin) 工作区。顶层包含 `src/`, `build/`, `devel/`。顶层 `src/CMakeLists.txt` 指向 ROS 的 toplevel cmake。
- 主要包（示例）：
  - `src/r300_function` — 导航、move_base 集成、多个 launch（例如 `launch/r300_navigation_indoor.launch`）和 RViz 配置。关键位置：`src/r300_function/launch/`、`src/r300_function/config/`。
  - `src/r300_bringup` — 设备/节点 bringup（CMakeLists 与 package.xml 为模板，主要依赖 roscpp/rospy）。
  - `src/scout_ros/*` — 机器人描述、基础节点、消息（`scout_msgs`）、bringup 与 launch；适用于模拟与实机启动。
  - `src/ugv_sdk` — SDK、示例/演示和测试（`demo/`、`test/` 子目录），常用于底层控制或样例代码。

## 构建与开发工作流（可直接使用的命令示例）
注：在运行下列命令前，请在包含 `src/` 的工作区根目录执行并确认有正确的 ROS 环境（例如在 Linux 上 source /opt/ros/<distro>/setup.bash）。

- 构建（最常见）
  - source 环境：
    - `source /opt/ros/melodic/setup.bash` (或相应 distro)
  - 构建：
    - `catkin_make` 或使用 catkin_tools：`catkin build`
  - 构建后载入开发环境：
    - `source devel/setup.bash`

- 启动导航示例（需适配雷达话题与 map 路径）
  - 例：启动室内导航（该 launch 已包含注释说明）
    - `roslaunch r300_function r300_navigation_indoor.launch scan_topic:=unilidar/laserscan map_file:=/path/to/your/map.yaml`
  - 注意：`r300_function/launch/r300_navigation_indoor.launch` 中明确要求只启动一次 `map_server`，并对 `scan` 做了 remap（默认 `unilidar/laserscan`）。

## 项目特定约定与常见模式（重要）
- 话题约定：雷达话题通常为 `unilidar/laserscan`（见 `r300_function` 的 launch），里程计话题通常为 `odom`，速度控制为 `/cmd_vel`，坐标系常用 `odom` 和 `base_link`。
- Launch 里常用 remap 与 include：`r300_navigation_indoor.launch` 会 include `move_base.launch` 并通过 arg 传参（示例见 `src/r300_function/launch/`）。
- 配置集中：RViz 配置位于 `src/r300_function/config/r300_navigation.rviz`，map 路径在 launch 中为硬编码默认值（务必改为本机路径或参数化）。
- 包模板：许多包的 `CMakeLists.txt` 和 `package.xml` 保持模板状态（注释多）。如果添加新节点，请按 catkin 模式添加 `add_executable` / `target_link_libraries` 并更新 `package.xml` 依赖。

## 集成点与外部依赖（agent 编码要点）
- 依赖：基于 ROS Melodic 风格（`buildtool_depend` 为 `catkin`，常用运行时依赖 `roscpp`, `rospy`）。在修改原生 C++/Python 节点时请同时维护 `package.xml` 依赖。
- 关键外部接口：
  - 雷达 -> topic `unilidar/laserscan`（若更换雷达，需同时更新 launch 中的 arg/remap）
  - AMCL/map_server -> 需要可用的地图文件（否则定位失败）
  - move_base -> 接受 `cmd_vel` 输入并依赖正
    确的 TF（`odom` -> `base_link`）

## 调试与验证指令（快速排查清单）
- 确认 topic：`rostopic list` / `rostopic echo <topic>`。
- 检查 TF：`rosrun tf tf_monitor` 或 `rosrun rqt_tf_tree rqt_tf_tree`。
- 可视化：`rosrun rviz rviz -d $(find r300_function)/config/r300_navigation.rviz`。
- 日志与节点：`rosnode list` / `rosnode info <node>`。

## 直接可用的代码示例（常见修改点）
- 若要将雷达话题改为 `scan`：在 launch 启动时传参：
  - `roslaunch r300_function r300_navigation_indoor.launch scan_topic:=/scan`
- 若新增 C++ 节点到 `r300_bringup`：
  - 在 `CMakeLists.txt` 中添加 `add_executable(...)` 并在 `package.xml` 添加相应依赖，随后 `catkin_make`。

## 参考文件（阅读这些文件会帮助 agent 立刻上手）
- `src/r300_function/launch/r300_navigation_indoor.launch` — 导航启动流程与参数说明（包括 map_server/AMCL/move_base/rviz）。
- `src/r300_function/config/r300_navigation.rviz` — RViz 配置。
- `src/scout_ros/*` — robot_description、bringup 与消息定义（`scout_msgs`）。
- `src/ugv_sdk/demo/` — 控制/示例代码与测试用例，适合复制参考。

## 结束与反馈请求
我已将该文件添加到仓库。请检查是否需要：补充更多具体运行示例（例如硬件驱动如何启动）、添加 CI/测试命令或合并本项目已有的 README 内容到此处。告知任何遗漏或你想强调的开发流程，我会据此迭代更新。
