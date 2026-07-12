# Lecture 5 参考答案

## ⚠️ 关于命名的说明：EKF vs KF

教程中的 `ExtendedKalmanFilter<N, M>` 模板类在**当前教学阶段**实现的是**标准线性卡尔曼滤波器（KF）**，
而非真正的扩展卡尔曼滤波器（EKF）。原因如下：

- **教程版**：F 矩阵和 H 矩阵都是常矩阵（匀速运动模型是线性的），
  没有实现非线性状态转移函数 f(x) 或观测函数 h(x) 的 Jacobian 线性化。
- **26_SP 版** (`tools/extended_kalman_filter.hpp`)：通过 `std::function` 注入
  自定义 f(x)/h(x)，支持非线性预测和更新，是真正的 EKF。


## 验证步骤

```bash
# 1. 先运行 Python 原型理解原理（★会同时导出 kf_1d_obs.csv 供 C++ 读取）
cd Day5-EKF/python_proto
python3 kf_1d_proto.py        # 一维 KF，生成 kf_1d_result.png + ../work/kf_1d_obs.csv
python3 ekf_sim.py             # 8维 EKF，生成 ekf_8dof_result.png

# 2. 编译运行 C++ 版本（Phase 1 读取 Python 导出的同一份观测数据）
cd Day5-EKF/work
mkdir build && cd build
cmake .. && make
./my_kf_1d                    # Phase 1: 一维 KF（与 Python 逐帧对比）

# 3. 取消 CMakeLists.txt 中 PHASE_2_ENABLED 注释 → 重新编译 → Phase 2: 8维 EKF
# 4. 取消 PHASE_3_ENABLED → 与仿真数据对比验证

# 5. 生成仿真数据并对比
cd tools
python3 sim_trajectory.py      # 生成 trajectory_circle_30fps.csv
python3 plot_compare.py trajectory.csv ekf_output.csv
```

## 常见错误

| 错误 | 正确写法 |
|------|---------|
| `P_ = F * P_ * F + Q` | `P_ = F * P_ * F.transpose() + Q` （F 需要转置） |
| `K = H * P_ * H^T + R` | 卡尔曼增益是 `P_*H^T*S^{-1}`，新息协方差才是 `S = H*P_*H^T+R` |
| 更新顺序: 先 update 再 predict | ★ 正确顺序: 先 predict 再 update |
| 协方差更新忘记 `(I - K*H)` | 公式 (5) 中的 `I` 是 N×N 单位矩阵，不是 M×M |

## Day5 → Day6 串联

- `my_ekf.hpp` 将被 Day6 `my_tracker.hpp` 通过 `#include "../../Day5-EKF/work/my_ekf.hpp"` 引用
- Day6 实例化 `ExtendedKalmanFilter<8, 3>`（8 维状态、3 维观测）
- 确保命名空间 `my_auto_aim` 一致，模板参数 `template<int N, int M>` 可正常实例化

## my_ekf.hpp 答案要点

模板类 `ExtendedKalmanFilter<N, M>` 的核心是两个方法：

```cpp
void predict(const StateMat& F, const StateMat& Q) {
    x_ = F * x_;                        // 公式 (1)
    P_ = F * P_ * F.transpose() + Q;    // 公式 (2)
}

void update(const ObsVec& z, const Eigen::Matrix<double, M, N>& H, 
            const ObsMat& R) {
    auto S = H * P_ * H.transpose() + R;  // 创新协方差
    GainMat K = P_ * H.transpose() * S.inverse(); // 公式 (3)
    ObsVec y = z - H * x_;               // 测量残差
    x_ = x_ + K * y;                     // 公式 (4)
    StateMat I = StateMat::Identity();
    P_ = (I - K * H) * P_;              // 公式 (5)
}
```

## 关键理解

- Q 越大 → 更信任观测 → 估计更噪声但响应快
- R 越大 → 更信任模型 → 估计更平滑但响应慢
- 8 维状态设计：位置+速度解释了为什么 EKF 能预测目标未来位置
- ★ Phase 1 使用方案A：Python 导出 CSV → C++ 读取同一份数据，确保逐帧可比
- ★ Phase 2 的仿真循环使用 `std::mt19937` + 固定种子(42)，保证每次运行可复现
- ★ Phase 3 读取 `sim_trajectory.py` 的 CSV 输出，使用 `std::stringstream` 解析
  （完整的 CSV 读取→EKF→CSV 写入骨架已提供在 work/main.cpp 中）
