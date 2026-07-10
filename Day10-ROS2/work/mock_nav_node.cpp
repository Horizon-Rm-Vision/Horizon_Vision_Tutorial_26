/**
 * Day10-ROS2/work/mock_nav_node.cpp —— 模拟导航 ROS2 节点
 *
 * #### Task 10-4（进阶）: 编写模拟导航程序 ########################
 *
 * 此节点模拟导航程序的行为：
 *   - 订阅 sp_msgs/msg/EnemyStatusMsg（自瞄发布的敌方目标信息）
 *   - 发布 geometry_msgs/msg/Twist（速度指令）
 *   - 在终端打印收到的敌方目标信息
 *
 * 编译前提：
 *   1. ROS2 Humble 已安装
 *   2. sp_msgs 已编译安装（参考 26_SP/sp_msgs/README.MD）
 *
 * 参考：
 *   - 26_SP io/ros2/publish2nav.cpp（发布 EnemyStatusMsg）
 *   - 26_SP io/ros2/subscribe2nav.cpp（订阅 Twist）
 *   - 26_SP tests/publish_test.cpp 和 subscribe_test.cpp
 *   - ROS2 官方 Tutorial: Writing a simple publisher and subscriber (C++)
 */

// TODO: 取消注释以使用 ROS2
// #include <rclcpp/rclcpp.hpp>
// #include <geometry_msgs/msg/twist.hpp>
// #include <sp_msgs/msg/enemy_status_msg.hpp>

#include <iostream>
#include <string>

// ================================================================
// #### Task 10-4: 实现 MockNavNode 类 ###########################
//
// 类设计:
//   class MockNavNode : public rclcpp::Node {
//       // 订阅者: 接收 EnemyStatusMsg
//       rclcpp::Subscription<sp_msgs::msg::EnemyStatusMsg>::SharedPtr enemy_sub_;
//       // 发布者: 发送 Twist 速度指令
//       rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
//       // 定时器: 周期性发布速度指令
//       rclcpp::TimerBase::SharedPtr timer_;
//   };
//
// ================================================================

int main(int argc, char** argv)
{
    std::cout << "========================================" << std::endl;
    std::cout << "  Lecture 10: 模拟导航 ROS2 节点" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << std::endl;
    std::cout << "此程序需 ROS2 环境。" << std::endl;
    std::cout << "当前以非ROS模式运行，仅打印提示。" << std::endl;
    std::cout << std::endl;
    std::cout << "完整实现步骤:" << std::endl;
    std::cout << "  1. source /opt/ros/humble/setup.bash" << std::endl;
    std::cout << "  2. 确保 sp_msgs 已安装:" << std::endl;
    std::cout << "     ros2 interface show sp_msgs/msg/EnemyStatusMsg" << std::endl;
    std::cout << "  3. 阅读 26_SP tests/publish_test.cpp" << std::endl;
    std::cout << "  4. 取消本文件中 ROS2 相关代码的注释" << std::endl;
    std::cout << "  5. 实现 MockNavNode 类" << std::endl;
    std::cout << "  6. 编译: colcon build 或 cmake .. && make" << std::endl;
    std::cout << "  7. 运行: ros2 run Day10-ROS2_ros2 mock_nav_node" << std::endl;
    std::cout << std::endl;
    std::cout << "核心功能:" << std::endl;
    std::cout << "  - 订阅 EnemyStatusMsg → 打印敌方位置/速度" << std::endl;
    std::cout << "  - 定时发布 Twist → 模拟导航发送速度指令" << std::endl;
    std::cout << "  - 运行 `ros2 topic echo` 验证消息流" << std::endl;

    // TODO: 取消注释以启动 ROS2 节点
    // rclcpp::init(argc, argv);
    // auto node = std::make_shared<MockNavNode>();
    // rclcpp::spin(node);
    // rclcpp::shutdown();

    return 0;
}

// ================================================================
// 参考实现（取消注释并填写 TODO）:
//
// class MockNavNode : public rclcpp::Node {
// public:
//     MockNavNode() : Node("mock_nav_node") {
//         // 订阅自瞄发布的敌方状态
//         enemy_sub_ = this->create_subscription<sp_msgs::msg::EnemyStatusMsg>(
//             "enemy_status", 10,
//             [this](const sp_msgs::msg::EnemyStatusMsg::SharedPtr msg) {
//                 enemy_callback(msg);
//             });
//
//         // 发布速度指令
//         cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
//             "cmd_vel", 10);
//
//         // 定时器: 每 100ms 发布一次速度指令
//         timer_ = this->create_wall_timer(
//             std::chrono::milliseconds(100),
//             [this]() { timer_callback(); });
//
//         RCLCPP_INFO(this->get_logger(), "MockNavNode 已启动");
//     }
//
// private:
//     void enemy_callback(const sp_msgs::msg::EnemyStatusMsg::SharedPtr msg) {
//         // TODO: 打印敌方位置、速度等信息
//         RCLCPP_INFO(this->get_logger(),
//             "收到敌方状态: pos=(%.2f, %.2f, %.2f) vel=(%.2f, %.2f, %.2f)",
//             msg->x, msg->y, msg->z,
//             msg->vx, msg->vy, msg->vz);
//     }
//
//     void timer_callback() {
//         // TODO: 发布模拟速度指令
//         auto twist = geometry_msgs::msg::Twist();
//         twist.linear.x = 1.0;   // 前进 1 m/s
//         twist.angular.z = 0.5;  // 旋转 0.5 rad/s
//         cmd_pub_->publish(twist);
//     }
//
//     rclcpp::Subscription<sp_msgs::msg::EnemyStatusMsg>::SharedPtr enemy_sub_;
//     rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
//     rclcpp::TimerBase::SharedPtr timer_;
// };
// ================================================================
