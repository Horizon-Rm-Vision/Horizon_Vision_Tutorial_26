# Lecture 4 参考答案

参考实现在 `Day4-Solver/work/solver.cpp` 中已由 TODO 引导完成。
若需对照，请查看 26_SP `tasks/auto_aim/solver.cpp` 的 `solve()` 函数。

## 自验证通过标准

运行 `./my_solver_test` 后应输出：
- 平移误差 < 5 cm ✗→✓
- 旋转向量误差 < 0.01
- 重投影误差 < 2 px
- 正交性误差 < 1e-6

## Python 原型

`Day4-Solver/python_proto/pnp_proto.py` 提供 Python 版验证脚本，
建议先用 Python 理解流程，再写 C++ 实现。

## 关键理解要点

1. solvePnP 的 IPPE 方法需要恰好 4 个点（装甲板四角点）
2. 旋转向量到欧拉角的转换不是唯一的，注意使用 ZYX 顺序
3. 坐标变换链的每一步都必须是正确的刚体变换
4. world2pixel 是验证位姿正确性的关键工具
