# Lecture 10 参考答案

ROS2 在 26_SP 中是可选的（`BUILD_SENTRY` 宏控制），
强调了核心算法不依赖特定通信框架的设计原则。

自定义消息 `sp_msgs` 作为自瞄与导航的接口契约，实现了两个程序的解耦。

## 模拟导航节点实现要点

1. 继承 `rclcpp::Node`
2. 订阅 `EnemyStatusMsg` 话题（从自瞄接收目标信息）
3. 发布 `Twist` 话题（向自瞄发送速度指令）
4. 定时器周期性发布指令

## 验证方法

```bash
# 终端 1: 运行模拟导航节点
ros2 run Day10-ROS2_ros2 mock_nav_node

# 终端 2: 查看敌方状态消息
ros2 topic echo /enemy_status

# 终端 3: 查看速度指令
ros2 topic echo /cmd_vel
```

参考 26_SP `tests/publish_test.cpp` 和 `tests/subscribe_test.cpp`。
参考脚手架: `Day10-ROS2/work/mock_nav_node.cpp`。
