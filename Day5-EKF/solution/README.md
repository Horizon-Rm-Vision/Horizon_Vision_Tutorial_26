# Lecture 5 参考答案

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

## 验证方法

1. 运行 Python 原型获得参考输出
2. 运行 C++ 版本比较结果（应在浮点精度内一致）
3. 用 tools/plot_compare.py 绘制 EKF 输出 vs 真实轨迹

## 关键理解

- Q 越大 → 更信任观测 → 估计更噪声但响应快
- R 越大 → 更信任模型 → 估计更平滑但响应慢
- 8 维状态设计：位置+速度解释了为什么 EKF 能预测目标未来位置
