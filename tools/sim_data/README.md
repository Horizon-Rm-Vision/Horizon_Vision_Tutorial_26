# tools/sim_data/ —— 仿真轨迹数据目录

## 说明

此目录用于存放由 `tools/sim_trajectory.py` 生成的仿真轨迹 CSV 文件，
供 Day5 (EKF) 和 Day6 (Tracker) 的 Phase 3 验证程序使用。

## 使用方式

在开始 Day5 Phase 3 或 Day6 Phase 3 之前，先运行：

```bash
cd Horizon_Vision_Tutorial_26
python3 tools/sim_trajectory.py                    # 默认：圆周运动，噪声 σ=0.02m, 30fps
python3 tools/sim_trajectory.py --motion linear    # 直线运动
python3 tools/sim_trajectory.py --noise 0.05       # 自定义噪声标准差
```

生成的文件将保存在此目录下，命名格式为：
- `trajectory_circle_highres.csv` — 高频仿真数据（dt=0.01s）
- `trajectory_circle_30fps.csv` — 降采样到相机帧率（30fps）
- `trajectory_linear_highres.csv` / `trajectory_linear_30fps.csv`

## CSV 列说明

| 列名 | 含义 | 单位 |
|------|------|------|
| t | 时间 | s |
| true_x, true_y, true_z | 真实位置 | m |
| true_vx, true_vy, true_vz | 真实速度 | m/s |
| true_yaw | 真实 yaw 角 | rad |
| true_omega | 真实角速度 | rad/s |
| obs_x, obs_y, obs_z | 带噪声的观测位置 | m |

## 验证方法

生成数据后，用 `tools/plot_compare.py` 对比 EKF 输出与真实轨迹：

```bash
python3 tools/plot_compare.py tools/sim_data/trajectory_circle_30fps.csv ekf_output.csv
```
