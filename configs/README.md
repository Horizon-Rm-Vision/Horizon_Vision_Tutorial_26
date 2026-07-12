# 相机参数使用说明

本教程涉及 **两套** 相机参数，切勿混用。

## 各 Day 应使用哪个？

```
Day4 Solver 独立验证 (verify_pnp)  →  任意一致参数即可（自投影自验证）
Day4 Solver 串联 Day3 + demo 视频  →  demo_camera.yaml  ★ 必须
Day4 Solver 串联 Day2 + demo 视频  →  demo_camera.yaml  ★ 必须
Day6 Tracker 模拟测试 (圆周运动)    →  任意一致参数即可（自投影自验证）
Day12 整合 + demo 视频              →  demo_camera.yaml  ★ 必须
Day12 整合 + 实车运行               →  my_camera_param.yaml（自标定）
```

