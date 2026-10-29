# 慧眼行动红方策略 · Swarm-Rescue

本仓库基于[赛事官方仿真框架](https://gitee.com/CJYLZS/swarm-eod)，加入了可运行的红方（`place`）初赛基线。无人机仅使用公开的传感器、通信和动作接口，在未知地图中探索、布设并提交地图；策略不读取仿真器真值。

- [红方策略与本地评测进度](PROGRESS.md)
- [安装说明](INSTALL.md)
- [原项目 Swarm-Rescue](https://github.com/emmanuel-battesti/swarm-rescue)
- [本 README 的原始英文版本](https://github.com/SHHDamn/swarm-eod-red-baseline/blob/92267b7615fc596d2dc6840bf3eccf22b411fc69/README.md)

## 快速开始

按 [`INSTALL.md`](INSTALL.md) 安装依赖后，在仓库根目录运行：

```powershell
$env:PYTHONPATH='src'
$env:PYTHONDONTWRITEBYTECODE='1'
python -m swarm_rescue.launcher --headless --competition -c config/competition_place_eval_plan.yml
```

红方入口是 `src/swarm_rescue/solutions/my_drone_eval.py`，当前选择 `MyDronePlaceBaseline`；策略实现在 `src/swarm_rescue/solutions/my_drone_place_baseline.py`。保留存活机接力；v09 在 v08 基础上进一步收缩近期融合图的噪声边缘。与 v08 的 10 组正常对照（7 提高、3 持平）及 6 组压力对照见 [`PROGRESS.md`](PROGRESS.md)，均为本地成绩。

正式参赛前须把 `src/swarm_rescue/solutions/team_info.yml` 中的示例队名、成员改为真实信息，并运行 `python scripts/check_submission.py`。`team_number` 保持官方默认整数；不要填入字母数字分享码，平台会分配队伍 ID。公开仓库保留示例个人信息，本地提交包使用真实信息。

禁区验证须检查实际实体和失效计数：当前 Map03 只保存计划中的 `zones_config`，没有据此创建禁区。普通报表中的禁区名称仅代表配置声明。使用 `scripts/benchmark_red_completion.py` 可记录实际区域数与禁用步数；显式注入区域的压力实验与普通计划成绩应分别记录，详见进度中的更正。

## 比赛任务与评分

同一个启动器通过 `team_mode` 区分任务：

- **红方（`place`）**：探索未知地图，使用 `place_bomb` 布设。初始库存由评测计划的 `initial_bombs_per_drone` 指定，仿真结束后输出炸弹位置 JSON。
- **蓝方（`rescue`）**：在同一地图几何结构上读取红方炸弹 JSON（`bombs_file`），寻找、抓取并运送炸弹到对应的处置中心。

```bash
.venv/bin/python -m swarm_rescue.launcher -c config/competition_place_eval_plan.yml
.venv/bin/python -m swarm_rescue.launcher -c config/competition_rescue_eval_plan.yml
```

`bombs_file: auto` 表示使用同一次运行中前一红方阶段的 JSON。评测达到最大仿真步数或最大实际运行时间时结束。算法过慢可能先触发实际时间上限。

### 红方评分

默认总分 100，由 `place_scoring` 配置权重；规则可能调整，应以评测源码为准。

- **地图（50%）**：通过 `submit_exploration_map()` 提交二值栅格。[`MapExplorationScorer`](src/swarm_rescue/tools/map_exploration_scoring.py) 与真实墙体栅格比较；处置中心、返回区和特殊区不是墙。墙面与深内部命中贡献分数，远离真实墙超过 10 像素的预测像素受惩罚，地图分为 `clamp(credit - penalty)`。未提交则本项为 0；时间奖励另算。
- **布设难度（20%）**：参考蓝队对各轮 `*_bombs.json` 评测。`difficulty = min(100, (100 - blue_score) × 2)`，再乘 `placed / expected`，其中 `expected = initial_bombs_per_drone × number_drones`。无效参考运行被排除；有效参考不足时此项标为 `partial`。
- **炸墙（20%）**：每枚炸弹的爆炸半径 `blast_radius` 默认是地图短边的 `0.12`。覆盖量按爆炸圆并集计算，边界墙默认权重 `0.5`，再除以同地图同炸弹数下的理论贪心最优覆盖量。
- **时间（10%）**：按步数线性计分，不超过上限一半为满分，到上限为 0；默认 `time_metric: timestep`。

多轮场景按轮次汇总。无人机返回区健康度**不计入红方分数**。

### 蓝方评分

处置炸弹占 60%，返回区健康度占 20%，效率占 20%。效率按步数计算；未处置完而运行至上限时，效率为 0。

## 仿真环境与接口

框架基于修改后的 [Simple-Playgrounds](https://github.com/mgarciaortiz/simple-playgrounds)，使用 Pymunk 和 Arcade。物体有质量与惯性；碰撞由物理引擎处理；每步进行感知、动作与通信。

### 坐标系

地图中心为 `(0, 0)`，`x` 向右、`y` 向上，单位为像素。地图尺寸是 `[width, height]`。朝向 `theta` 范围 `[-π, π]`，逆时针增加；`theta = 0` 时朝右。

### 传感器

- **激光雷达**：`DroneLidar` 位于 `simulation/ray_sensors/drone_lidar.py`；360°、181 条射线、最大 300 像素，距离有高斯噪声。首尾射线分别位于 `-π` 和 `π`。用法见 `my_drone_lidar_communication.py`；`draw_lidar_rays=True` 可显示射线。
- **语义传感器**：`DroneSemanticSensor` 为 360°、35 条射线、最大 200 像素；数据包含 `distance`、`angle`、`entity_type`、`grasped`。比赛中能识别炸弹、处置中心和其他无人机，**不能识别墙**。墙体检测与避障应使用激光雷达。距离有高斯噪声；`draw_semantic_rays=True` 可显示射线。
- **GPS**：`DroneGPS` 输出地图中心坐标，带一阶自回归噪声；禁 GPS 区域内不可用。
- **罗盘**：`DroneCompass` 输出逆时针为正的朝向，范围 `[-π, π]`，带一阶自回归噪声；禁 GPS 区域内不可用。
- **里程计**：`DroneOdometer` 给出上一步位移 `dist_travel`、相对位移角 `alpha` 和朝向变化 `theta`，分别带高斯噪声。可在无 GPS 时积分估计位姿，但会累积误差。[示意图](img/odom.png)。

GPS、罗盘和里程计定义见 `src/swarm_rescue/simulation/drone/drone_sensors.py`。`GuiSR(enable_visu_noises=True)` 可显示噪声和里程计估计轨迹。

### 通信与动作

无人机可与 250 像素范围内的队友通信，每步发送和接收消息。通过 `define_message_for_all()` 定义消息，在通信组件的 `received_messages` 中读取消息。

`control()` 每步返回动作字典。`forward`、`lateral`、`rotation` 均为 `[-1, 1]` 的浮点数，分别控制前进、横移、旋转。`grasper` 为二值动作：`1` 抓取、`0` 释放。红方额外使用二值 `place_bomb`；由 `0` 变为 `1` 的上升沿会在有库存时尝试布设一枚炸弹。用 `carried_bombs_count()` 查询剩余库存。

```python
command = {
    "forward": 1.0,
    "lateral": 0.0,
    "rotation": 0.0,
    "grasper": 0,
    "place_bomb": 1 if self.carried_bombs_count() > 0 else 0,
}
```

蓝方处置流程：识别 `TypeEntity.BOMB`，靠近后抓取，运到其对应的处置中心并释放，或携带炸弹与中心碰撞。携带中的炸弹对该无人机的语义传感器透明。静态炸弹不移动；动态炸弹沿预设路径运动。处置中心语义类型为 `TypeEntity.DISPOSAL_CENTER`，只有炸弹绑定的中心能完成处置。示例见 `examples/example_semantic_sensor.py` 与 `examples/example_moving_bomb.py`。

返回区是蓝色区域，不会被传感器直接看到；可通过 `is_inside_return_area` 判断。若地图没有返回区，蓝方健康分使用地图内所有无人机的健康度。

### 特殊区域

特殊区域对普通传感器不可见：

- `NO_COM_ZONE`：切断无线通信。
- `NO_GPS_ZONE`：GPS 和罗盘失效，需依靠里程计。
- `KILL_ZONE`：进入后立即摧毁无人机。

### 旧版 API 名称

旧教程可能使用救援伤员命名；当前代码使用炸弹处置命名：

- `WoundedPerson` → `Bomb`
- `RescueCenter` → `DisposalCenter`
- `TypeEntity.WOUNDED_PERSON` → `TypeEntity.BOMB`
- `TypeEntity.RESCUE_CENTER` → `TypeEntity.DISPOSAL_CENTER`
- `grasped_wounded_persons()` → `grasped_bombs()`
- `number_wounded_persons` → `number_bombs`
- JSON `"type": "rescue_center"` → `"type": "disposal_center"`
- `character_v2.png` / `rescue_center.png` → `bomb.png` / `disposal_center.png`

资源在 `src/swarm_rescue/resources/`，修改图像生成逻辑后可运行 `.venv/bin/python -m swarm_rescue.tools.generate_resource_sprites`。

## 评测计划与运行

YAML 计划定义地图、轮数、无人机数、特殊区域和输出选项，不必修改 Python 代码。本仓库还提供 `config/local_place_test_plan.yml` 用于 Map02/Map03 本地测试。

```yaml
stat_saving_enabled: true
state_recording_enabled: true
video_capture_enabled: false

evaluation_plan:
  - map_name: Map04
    team_mode: place
    nb_rounds: 2
    config_weight: 1
    number_drones: 5
    initial_bombs_per_drone: 2
    zones_config: [NO_COM_ZONE, NO_GPS_ZONE]
```

`stat_saving_enabled` 保存统计并生成 PDF；`state_recording_enabled` 生成可回放的 `.npz`（默认开启）；`video_capture_enabled` 生成 `.avi`（默认关闭）。`map_name` 指定地图，`nb_rounds` 指定轮数，`config_weight` 指定汇总权重，`zones_config: []` 表示不启用特殊区。统计与录制一般写入 `~/results_swarm_rescue`。状态录制通常约 150 KB/轮；AVI 视频可达每轮 50–200 MB。

```bash
python src/swarm_rescue/launcher.py --config config/competition_place_eval_plan.yml
python src/swarm_rescue/launcher.py --config config/competition_rescue_eval_plan.yml
.venv/bin/python -m swarm_rescue.launcher --replay <recording.npz>
```

回放窗口初始暂停；空格播放/暂停，左右方向键跳转约 30 帧，上下方向键调速，`Q` 退出。普通仿真中，`Q` 结束当前轮，`E` 结束全部轮次。

### 无窗口运行

增加 `--headless` 或 `-H` 可不显示窗口。部分 Linux 环境中，Arcade 仍需要 X11 创建 OpenGL 上下文；没有显示服务时可使用虚拟帧缓冲：

```bash
sudo apt-get install xvfb
xvfb-run -s "-screen 0 1920x1080x24" python src/swarm_rescue/launcher.py --headless --config config/competition_rescue_eval_plan.yml
```

### 在 Python 中创建计划

```python
from swarm_rescue.simulation.reporting.evaluation import EvalConfig, EvalPlan
from swarm_rescue.simulation.elements.sensor_disablers import ZoneType

easy_config = EvalConfig(map_name="MyMapEasy01")
hard_config = EvalConfig(
    map_name="Map04",
    zones_config=(ZoneType.NO_COM_ZONE, ZoneType.NO_GPS_ZONE, ZoneType.KILL_ZONE),
    nb_rounds=3,
    config_weight=2,
)
plan = EvalPlan()
plan.add(easy_config)
plan.add(hard_config)
```

其他示例见 `src/swarm_rescue/maps/map_01.py` 至 `map_05.py`。

## 项目目录与编程

- `src/swarm_rescue/solutions/`：参赛策略和示例；无人机代码放在这里。
- `src/swarm_rescue/maps/`：地图定义，继承 `MapAbstract`。
- `src/swarm_rescue/simulation/`：仿真器核心（`drone`、`elements`、`gui_map`、`ray_sensors`、`reporting`、`utils`）；参赛策略不应修改。
- `src/swarm_rescue/tools/`：地图转换和测量工具。
- `examples/`：独立演示程序；`config/`：评测计划。

策略类继承 `DroneAbstract`，实现 `define_message_for_all()` 与 `control()`。同一份类代码会在各无人机实例中分别运行。比赛策略只能使用传感器和通信数据，例如 `measured_gps_position()`、`measured_compass_angle()`；`true_position()`、`true_angle()` 及直接读取真实 `position`、`angle` 仅供调试，不可作为策略输入。

当前红方入口 `drone_class_for_mode("place")` 返回 `MyDronePlaceBaseline`；蓝方仍使用官方示例。启动器是 `src/swarm_rescue/launcher.py`。

示例文件包括：`my_drone_random.py`（随机运动）、`my_drone_lidar_communication.py`（雷达与通信）、`my_drone_motionless.py`（静止模板）、`examples/example_display_lidar.py`（雷达曲线）、`examples/example_com_disabler.py` 和 `example_gps_disablers.py`（失效区）、`example_mapping.py`（建图）、`example_pid_rotation.py` 与 `example_pid_translation.py`（控制）、`example_semantic_sensor.py`（抓取处置）、`example_moving_bomb.py`（动态炸弹）。`example_keyboard.py` 演示键盘控制；方向键移动转向，`Shift+方向键` 横移，`W` 抓取，`L/S` 切换射线，`P/C/M` 查看位姿、通信和消息，`Q` 退出，`R` 重置。

`GuiSR` 可开启 `draw_zone`、`draw_lidar_rays`、`draw_semantic_rays`、`draw_gps`、`draw_com`、`print_rewards`、`print_messages`、`use_keyboard`、`use_mouse_measure` 和 `enable_visu_noises`。`filename_video_capture`、`filename_state_recording` 指定录制文件。需要查看帧率时，可在 `gui_sr.py` 设置 `DISPLAY_FPS = True`。`DroneAbstract` 可重写 `draw_top_layer()`、`draw_bottom_layer()` 绘制调试覆盖层。

## 提交与调试

正式提交内容应限于 `src/swarm_rescue/solutions/` 中的策略代码、填写正确的 `team_info.yml` 和新增依赖列表。不要把仿真器或本地评测输出打入提交包。工作区及最终 `teamNNN_evalstep.zip` 都应运行提交检查；详情见 [提交检查与打包](doc/participants/03-提交检查与打包.md)。

```bash
./check_submission.sh
python scripts/check_submission.py
```

蓝方测试时可用图形界面编辑炸弹布局，并将导出的 JSON 作为 `bombs_file` 使用：

```bash
python src/swarm_rescue/launcher.py \
  -c config/competition_rescue_eval_plan.yml \
  --manual-bomb-edit \
  --manual-bombs-in path/to/input_bombs.json \
  --manual-bombs-out path/to/output_bombs.json
```

手动模式中左键添加、右键删除最近炸弹，`I` 导入、`J` 导出、`K` 清空、回车确认。导出 JSON 包含 `map_name`、`zones_config`、`bombs`，可通过 YAML 的 `bombs_file` 或命令行 `--bombs-file` 再次加载。

## 创建自定义地图

自定义地图可复现问题并测试不同墙体、炸弹及特殊区域。通常在 `src/swarm_rescue/maps/` 新建 `map_<name>.py`（继承 `MapAbstract`）和 `walls_<name>.py`（墙体辅助函数）：

1. 绘制 PNG：墙体纯黑、厚约 8–12 像素；炸弹亮黄，处置中心纯红。
2. 在 `image_to_map.py` 设置 `img_path` 并运行。工具会生成 `generated_code.py`，在控制台打印地图初始化代码。
3. 把墙体函数复制到 `walls_<name>.py`；把打印的 `self._size_area`、`self._disposal_center`、`self._disposal_center_pos`、`self._bombs_pos` 原样放入地图类。这些值须与转换结果一致。
4. 在 `build_playground()` 中添加墙体、无人机起点、返回区等；转换器不会自动设置起点。
5. 用 `src/swarm_rescue/tools/check_map.py` 检查布局，再运行短时仿真。若颜色识别失败，检查转换器的 HSV/亮度阈值。

## 联系方式

原项目代码或文档问题可联系原作者：emmanuel . battesti at ensta . fr。赛事规则应以官方发布的信息和当前评测源码为准。
