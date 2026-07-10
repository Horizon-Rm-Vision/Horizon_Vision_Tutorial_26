# Lecture 10: ROS2 通信集成

本 Lecture 以阅读和理解 26_SP `io/ros2/` 源码为主。

## 学习路径

1. 编译安装 `sp_msgs` 自定义通信包
2. 阅读 `io/ros2/ros2.cpp/hpp`——ROS2节点管理
3. 阅读 `io/ros2/publish2nav.cpp/hpp`——向导航发布敌方信息
4. 阅读 `io/ros2/subscribe2nav.cpp/hpp`——订阅导航指令
5. 编译哨兵模式: `cmake .. -DBUILD_SENTRY=ON`
6. （进阶）编写模拟导航节点

## 核心数据流

```
导航程序 ←(EnemyStatusMsg)← 26_SP自瞄 ←(串口)← 电控
导航程序 →(Twist/SentryCmd)→ 26_SP自瞄 →(串口)→ 电控
```

参考 26_SP `sp_msgs/` 和 `io/ros2/` 目录。
