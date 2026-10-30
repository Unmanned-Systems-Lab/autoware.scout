# Scout HT 融合图启动与验证

## 当前完成范围

已实现计划书阶段 A：独立 ROS 包 `scout_ht_traversability`，将原始 costmap 与 ELE Scout 的 HT 八方向概率融合，发布黑白结果和诊断。
**本文的独立入口仍只观察，不修改规划器输入，也不发送底盘或自动驾驶指令。**
2026-10-26 已增加可选的阶段 B 停车插件和阶段 C 局部采样路径检查，
通过 Scout 的 `scout_ht_enforce:=true` 启用，见 [SCOUT_HT_PLANNING.md](SCOUT_HT_PLANNING.md)。尚未实车验收。

每个格按“小车当前位置指向该格”的方位角选择 HT 通道。原始 costmap 禁止则保持黑色；原始允许且所选方向 HT 通过才为白色。车辆移动后重新计算，可以恢复之前仅因 HT 变黑的格，不把结果回写原始 costmap。
2026-10-28 起，Scout 的未观测、尚无有效 HT 结论和 HT 覆盖外区域采用可通行先验。
原始 costmap 的障碍/未知格仍优先禁止，已有有效 HT 低评分仍禁止。
输入缺失/过期、非法数据及 TF 错误仍使输出失效。
融合格为 `0=白、100=黑`；原因图的 `10=PRIOR_FREE` 区分先验放行与 HT 实测通过。

## 构建

本机已完成一次构建；新增工作区使用：

```bash
cd /mnt/nvme0n1/autoware/autoware.scout
source ./setup_env.bash
colcon build --symlink-install --packages-select scout_ht_traversability
```

ELE 源码的独立发布目标为 `git@github.com:CQULzz/eletranskit_scout.git`，依赖清单为 `repositories/scout-external.repos`，固定提交 `5163982d0faef28e84c9b00dc9d7367707175871`。该提交已发布在 `handover/20261030-source`；当前 GitHub 身份有读取权限即可导入：

```bash
mkdir -p ../scout_dependencies
vcs import ../scout_dependencies < repositories/scout-external.repos
```

在独立的 `../scout_dependencies/ele_scouts_kit` 工作区按其 README 构建，不导入 Autoware 的 `src`，也不把两套代码合并成一个仓库。此处 `SOURCE_VERSIONS.repos` 继续记录原始 Autoware 来源，不替代这个可选依赖清单。
启动脚本优先使用环境变量 `SCOUT_HT_ELE_WORKSPACE` 指定路径，其次使用已构建的上述导入路径，最后兼容本机原路径 `/home/agx/ele_scout_kit_github_20260827`。本机无需为了发布而移动正在使用的 ELE 工作区。
启动脚本核对实际选中的 HT 包前缀，避免误用本机另外两套同名包。沿用终端的 ROS 域和 DDS 配置，与正在运行的 Autoware 保持一致。

## 两种启动方式

Autoware 继续按 `SCOUT_STARTUP.md` 启动，不必为这个旁路节点重启。

只启动融合节点，要求另有节点发布 `/scout/ht/raw`：

```bash
cd /mnt/nvme0n1/autoware/autoware.scout
./start_scout_ht.bash rviz:=true
```

如果 ELE 高程和 HT 尚未运行，可在另一个终端一并启动它们的感知部分：

```bash
cd /mnt/nvme0n1/autoware/autoware.scout
./start_scout_ht.bash start_elevation:=true start_ht:=true rviz:=true
```

此入口不启动底盘、Fixposition、雷达驱动或另一套控制器。高程节点使用 Autoware 已裁切的 `/sensing/lidar/concatenated/pointcloud`、`map` 和 `base_link`；不会重复扩大自车裁切，不发布修正 TF，不用启动地面先验。
全局多帧高程按 Fix 原点和外参绑定到独立数据库，同原点重启会恢复积累数据，原点变化另存。
HT 的 10×10 m 输入从该全局图裁剪，详见 [全局高程说明](SCOUT_GLOBAL_ELEVATION.md)。
ELE 的局部全局高程图仍输出 `/elevation_mapping_node/elevation_map_global_local` 的 `elevation` 层，输入 HT 的分辨率为 0.04 米；HT 输出隔离为 `/scout/ht/raw`，显式开启八方向概率、关闭保守合并层。
**只有真实传感器、高程与坐标关系检查通过后，这个启动才有有效现场数据。本轮已验证模型和合成数据链路，未启动现场高程建图或宣称其已通过实车验证。**

## 原始 Costmap 与阈值

2026-10-27 已将原始 costmap 分辨率改为 **0.04 m**，范围保留 **70×70 m**，
对应 **1750×1750** 格。融合输出沿用相同几何，因此也是 4 厘米格子。
配置位于 `autoware_launch/config/planning/scenario_planning/common/costmap_generator.param.yaml`。
生成器需要重新启动才能读取该配置；后续正常启动即使用新分辨率。
HT 查询仅计算其局部覆盖窗口；窗口外按当前可通行先验处理，完整输出范围不会缩小。
4 厘米格子不会扩大 HT 的有效覆盖范围，也不会增加传感器本身的观测精度。

默认原始话题：`/planning/scenario_planning/parking/costmap_generator/occupancy_grid`。
原版本生成器只由泊车场景激活。2026-10-26 Scout 已默认开启
`publish_lane_driving_costmap:=true`，让同一个生成器在车道场景发布；
不改变原来的话题名、数据源或代价含义。2026-10-27 起，Scout 的此开关同时设置
`publish_continuously=true`，无需路线或场景即可发布真实 costmap；缺地图、有效传感器输入或 TF 时仍会等待。
其他未启用新参数的 Autoware 配置保持旧行为；仅设 `activate_by_scenario=false` 仍不是持续发布开关。

默认 `obstacle_threshold=100` 来自本机 `freespace_planner.param.yaml`，按 `值 >= 阈值` 禁止，负数和超出 0..100 的值为未知。其他规划消费者可能采用不同阈值，**不能把本默认值当作所有模块的统一通行标准**。
显式选择另一个实际存在的 costmap 时，需同时指定与该消费者一致的阈值，例如：

```bash
./start_scout_ht.bash base_costmap_topic:=/实际原始costmap话题 obstacle_threshold:=100
```

该命令仅示意覆盖参数，不要将话题替换为融合输出，也不要把感知占据栅格当成相同语义的 costmap 来绕过缺失输入。
其他配置位于 `src/perception/scout_ht_traversability/config/fusion.yaml`，修改后只需重启本旁路节点，不需重启 Autoware。
默认 HT 阈值 0.15 沿用 ELE 诊断配置，窗口支持比例 0.30 沿用其实车 YAML，两者都不代表已完成本轮场地标定。
参考点偏移 0.4825 米仅适用于当前未做参考点转换的 ELE 原始模型输出；如接入已转换的地图，必须避免再次偏移。

## 查看结果

2026-10-27 室外启动确认高程和八方向 HT 均有约 5 Hz 的真实输出，
修正了 ELE Reliable 点云订阅与 Autoware Best Effort 发布不兼容的问题。
完整启动命令、订阅检查和仍待验证的部分见
[完整启动记录](validation/scout_full_20261027_150740/REPORT.md)。
后续已将 Scout 的真实 costmap 发布与路线/场景触发分离：启动后只要地图、点云和 TF 有效，
即可查看原始 costmap 和融合图。`WAITING_BASE` 表示尚未收到真实原始图，仍需检查输入，
不能用假路线或假自由空间来消除等待。

```bash
cd /mnt/nvme0n1/autoware/autoware.scout
source ./setup_env.bash
ros2 topic echo --once /scout/traversability/status
ros2 topic echo --once /scout/traversability/fused_costmap --field info
```

RViz 配置已随包提供；也可以在当前窗口 Add → Map，话题设为 `/scout/traversability/fused_costmap`，Color Scheme 设为 `map`，Fixed Frame 设为 `map`。
`READY` 表示输入通过当前时效/接口检查，不代表全图可通行。`WAITING_BASE` 表示原始 costmap 未收到，`WAITING_HT` 表示 HT 未收到，`STALE_*` 表示数据过期；不伪造白色地图来消除这些提示。
未收到任何有效原始 costmap 时不发布没有真实几何的假地图；收到过有效几何后，如输入失效则发布该范围全黑结果，避免保留上一帧白色显示。

调试话题：`/scout_costmap_ht_fusion/selected_direction` 为选中几何方向 0..7，-1 为未查询。
`/scout_costmap_ht_fusion/reason_map` 编码：0实测通过、1原始禁止、2原始未知、3HT无效、4HT低分、5过近、6过期、7TF错误、8输入错误、9等待输入、10先验可通行。这些数值不是占用概率。
`/scout/ht/raw` 新增 `ht_observed` 与 `ht_prior_free`，先验不会伪造成模型预测概率。
`unobserved_is_free=false` 可恢复旧的缺测禁止策略；高程的先验显示由
`global_map_publish_unobserved_prior` 控制，已观测的高程后验不会被重置。

## 验证记录

- 新包 `colcon build` 成功。
- 2026-10-23：原有 29 项加新增 30 项 Python 测试，共 59 项通过，含实际 DDS 收发测试。
- ELE 模型 CUDA preflight 通过，权重 SHA256 为 `0a79b865d012aa633fbb9913fc4031908dfe188ec9e161ef5a3fbb34a3f8c4d4`。
- 独立 ROS 域的合成高程通过真实 ELE HT 模型进入融合节点，输出验证为 `[100,0,0,0]`：原始禁止格不被 HT 清除。测试进程已停止，不连接实车控制。
- 本机纯融合函数测试：233×233 原始 costmap、250×250 HT、20 次，P50 约 101 ms、P95 约 124 ms；不包含 HT 推理、ROS 传输与地图解码，不代表整条链路延迟。
- 上述为阶段 A 的记录。后续规划接入与回归见 `SCOUT_HT_PLANNING.md`；
  真实地形标定、现场高程链路和实车运动验收仍未完成，不能据此声明车辆可以依靠融合图行驶。

回归命令（独立测试域，不修改用户启动环境）：

```bash
source ./setup_env.bash
ROS_DOMAIN_ID=197 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  python3 -m unittest discover -s tests -v
```

停止本旁路启动终端用 `Ctrl+C`；原有 Autoware 规划、控制和原始 costmap 不受其退出影响。
