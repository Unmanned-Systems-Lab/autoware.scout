# Scout HT 规划接入

日期：2026-10-26。本轮接入阶段 B 和阶段 C 的局部候选筛选，未完成实车验收。
融合图本身的生成方式见 [SCOUT_HT_STARTUP.md](SCOUT_HT_STARTUP.md)。

## 本轮实现

- 保留原来的 costmap 生成器、点云输入、数值含义和话题名：
  `/planning/scenario_planning/parking/costmap_generator/occupancy_grid`。
  名称虽然含 `parking`，现在 Scout 车道场景也可发布，不另起第二套地图。
- `real_scout.launch.xml` 默认 `publish_lane_driving_costmap:=true`；
  同时启用 `publish_continuously`，原始 costmap 无需路线或场景触发。
  缺 Lanelet2 地图、有效点云或车辆 TF 时不发布伪造的自由空间。
  新发布模式检查所用传感器和车辆 TF 的年龄，默认上限 0.5 秒；
  点云/目标变换使用原始时间戳，输出沿用最旧输入时间，不把旧点云刷新成新地图。
- 新增 `scout_ht_planning::VelocityModule`，通过原有 motion velocity planner
  插件接口插入停车点和接近段速度上限，最后由原有轨迹处理链应用。
  没有新增底盘命令发布者。
- 局部路径选择复用 `autoware_path_sampler`，只在启用时筛除碰到融合黑格的候选。
  原有曲率、检测目标和可行驶边界检查继续保留；启用后车身必须位于可行驶边界内。
  复用旧路径前先更新本轮边界和障碍约束，不能沿用上一轮的可行驶范围。
  直接复用路径、重新采样候选和最终回退路径均检查同一份冻结地图。
- 车辆 footprint 来自 Scout 车辆参数。两采样姿态之间使用凸包及转动弦高裕量
  覆盖连续车身扫掠，不能只靠中心线或两个端点是否为白色判断。
- 仅明确的融合值 `0` 可通过；其他值、未知、地图范围外均拒绝。
  候选检查至少覆盖当前制动前视范围；最终速度插件检查整条输出轨迹，
  在未验证远区前插入停车点，不能把一段白色前缀理解为整条路线都可执行。
- 地图、定位、坐标系或时序无效时不回退为无 HT 放行；求解完成前再次检查地图时效。
  同一原点下的新 costmap 自动进入下一规划周期，不需要逐张图重启。

## 构建

```bash
cd /mnt/nvme0n1/autoware/autoware.scout
source ./setup_env.bash
colcon build --symlink-install --executor sequential \
  --packages-select scout_ht_planning scout_ht_traversability \
  autoware_path_sampler autoware_costmap_generator tier4_planning_launch autoware_launch \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source ./setup_env.bash
```

本次修改了原生规划及 costmap 组件，首次加载新二进制需要重启相应容器。
这和正常运行时收到一张新 costmap 不同，后者不需要重启。

## 模式与接线

保持原启动指令，`scout_ht_enforce` 默认 `false`，不会自动使用 HT 改变路径和速度。
本轮新增的车道 costmap 发布默认开启；若需要恢复原来的泊车发布条件，
在未启用 HT 时指定 `publish_lane_driving_costmap:=false`。

完成方向、阈值、坐标、时效及制动检查后，可在原 Scout 启动指令中增加：

```bash
scout_ht_enforce:=true
```

该开关默认选择 `scout_ht_terrain` 规划 preset，并加载 HT 速度插件及采样路径检查。
不要同时覆盖成其他 `planning_module_preset`。HT 模式不加载未接入融合约束的
泊车路径规划器，并关闭 freespace bypass coordinator，但仍保留同一个 costmap 生成器。
这是车道内局部路径筛选，不是取消 Lanelet2 约束的自由导航，也不是改造了 MPT 优化器。

另一个终端按阶段 A 的独立入口提供高程、HT 和融合图：

```bash
cd /mnt/nvme0n1/autoware/autoware.scout
./start_scout_ht.bash start_elevation:=true start_ht:=true rviz:=true
```

它仍不启动底盘、Fixposition、雷达驱动或 HT_Explo 的独立控制器。
两终端使用相同 ROS 域、时钟和兼容 RMW；原始 costmap 绝不能改成融合输出，避免反馈。
启用约束而没有上述输入时会停车，这是预期行为。

## 参数与诊断

规划器参数前缀为 `scout_ht.`，启动时设置后不可在线修改：

| 参数 | 默认值 | 含义 |
|---|---|---|
| `costmap_topic` | `/scout/traversability/fused_costmap` | 必须是二值融合图，非原始感知占据概率 |
| `map_timeout` | 0.5 s | ROS 时间年龄和接收后墙钟年龄上限 |
| `stop_margin` | 0.5 m | 从保守扫掠碰撞位置向后退的距离 |
| `deceleration` | 0.5 m/s² | 制动预算初值，尚未实测标定 |
| `latency` | 0.3 s | 额外反应延迟，计算时还加入融合数据年龄 |
| `minimum_lookahead` | 2.0 m | 候选路径至少检查的前视距离 |

这些数值不是车辆制动能力保证；不足以在物理上停车时，零速度约束也不能消除惯性。
保留现场急停和原有动态障碍/控制检查。当前执行消费要求 `map` / `base_link`，
不会通过静默改 frame 名字来接受不同坐标。

速度插件的状态和停车墙位于 motion velocity planner 的私有命名空间：

- `~/scout_ht/status`：`CLEAR`、`BLOCKED_CELL`、`OUTSIDE_MAP`、缺图/过期等原因。
- `~/scout_ht/virtual_wall`：实际插入的停车位置，可在 RViz 以 MarkerArray 显示。

全局融合诊断仍为 `/scout/traversability/status`，逐格原因图仍为
`/scout_costmap_ht_fusion/reason_map`。2026-10-28 起，Scout 开启未观测区域可通行先验，
原因值 10 表示先验放行；原始障碍、有效 HT 低评分仍禁止。
已有 HT 观测证据但方位角接近零的格仍标为过近；未观测的车下格采用先验。
规划器仍只接受实际融合输出中的 0 值，缺图、过期、非法数据和地图范围外仍拒绝。

## 验证边界

本轮只运行隔离域中的合成数据、原生算法和接口回归，未启动实车、设备或运动请求。
未验证真实地形方向标定、0.15 HT 阈值的执行可靠性、最坏并发延迟、坡面滑移、
实际制动能力或整套带 GPU 感知的 Autoware 启动。以上仍是现场启用前的必需检查。
测试命令和结果见 [本轮验证记录](validation/scout_ht_planning_2026-10-26/REPORT.md)。
当前对融合图统一做车身扫掠；如果模型标签本身已含整车几何，需要在窄道回放中
核对重复保守性，不能将无路或停车一律归因为实际物理障碍。
