#!/bin/bash
# ================================================================
# my_autostart.sh —— Day13 部署脚本（开机自启动 + 看门狗 + 串口赋权）
#
# 用途：在 Ubuntu 系统开机时自动启动你的整合自瞄 Pipeline。
#
# #### Task 13-5a: 编写开机自启动脚本 ############################
#
# 使用方法：
#   1. 将此脚本复制到你的 build 目录
#   2. 修改下面的路径配置
#   3. 赋予执行权限: chmod +x my_autostart.sh
#   4. 测试运行: ./my_autostart.sh
#   5. 设置开机自启动（方法选一）:
#      a. 添加到 crontab: @reboot /path/to/my_autostart.sh
#      b. 创建 .desktop 文件放入 ~/.config/autostart/
#      c. 添加到 /etc/rc.local（需要 root）
#
# 参考：
#   - 26_SP autostart_V1.sh / autostart_V2.sh
#   - Training-26-5 你自己编写的自启动脚本
# ================================================================

# ============================================================
# 配置区域 —— 根据你的环境修改以下路径
# ============================================================

# 你的整合程序的可执行文件路径
PROGRAM_PATH="/home/ad/Training-26-Lecture1/Horizon_Vision_Tutorial_26/Day13-Integration2/work/build/my_full_pipeline"

# 配置文件目录
CONFIG_DIR="/home/ad/Training-26-Lecture1/Horizon_Vision_Tutorial_26/Day13-Integration2/work/build"

# 串口设备路径（用于赋权）
SERIAL_DEVICE="/dev/ttyUSB0"

# 日志文件路径
LOG_FILE="/tmp/my_pipeline.log"

# 看门狗检查间隔（秒）
WATCHDOG_INTERVAL=2

# ============================================================
# ★ 函数定义 ★
# ============================================================

# --- 串口赋权 ---
grant_serial_permission() {
    echo "[autostart] 为串口赋权..."
    
    # 方法 1: chmod（简单但非永久）
    if [ -e "$SERIAL_DEVICE" ]; then
        sudo chmod 666 "$SERIAL_DEVICE"
        echo "[autostart] 已赋权 $SERIAL_DEVICE"
    fi
    
    # 方法 2: 对所有 ttyUSB 设备赋权
    for dev in /dev/ttyUSB*; do
        if [ -e "$dev" ]; then
            sudo chmod 666 "$dev"
            echo "[autostart] 已赋权 $dev"
        fi
    done
    
    # 方法 3: 使用 udev 规则（永久，推荐）
    # 将以下内容写入 /etc/udev/rules.d/99-usb-serial.rules:
    #   KERNEL=="ttyUSB*", MODE="0666"
    # 然后运行: sudo udevadm control --reload-rules
}

# --- 启动程序 ---
start_program() {
    echo "[autostart] 启动 Pipeline: $PROGRAM_PATH"
    cd "$CONFIG_DIR" || exit 1
    
    # 启动程序，输出重定向到日志
    $PROGRAM_PATH >> "$LOG_FILE" 2>&1 &
    PROGRAM_PID=$!
    echo "[autostart] Pipeline PID: $PROGRAM_PID"
}

# --- 看门狗：监控程序是否运行 ---
watchdog() {
    while true; do
        sleep "$WATCHDOG_INTERVAL"
        
        # 检查进程是否存活
        if ! kill -0 "$PROGRAM_PID" 2>/dev/null; then
            echo "[watchdog] $(date): Pipeline 已退出 (PID $PROGRAM_PID)，重新启动..."
            echo "[watchdog] $(date): 最后 10 行日志:"
            tail -10 "$LOG_FILE"
            
            # 重新启动
            start_program
        fi
    done
}

# --- 清理函数（脚本退出时调用）---
cleanup() {
    echo "[autostart] 收到退出信号，清理..."
    if [ -n "$PROGRAM_PID" ]; then
        kill "$PROGRAM_PID" 2>/dev/null
        echo "[autostart] 已终止 Pipeline (PID $PROGRAM_PID)"
    fi
    exit 0
}

# ============================================================
# ★ 主流程 ★
# ============================================================

echo "========================================"
echo "  Horizon Vision Pipeline 自启动脚本"
echo "  启动时间: $(date)"
echo "========================================"

# 捕获退出信号
trap cleanup SIGINT SIGTERM

# 1. 串口赋权
grant_serial_permission

# 2. 检查程序文件是否存在
if [ ! -f "$PROGRAM_PATH" ]; then
    echo "[autostart] 错误: 找不到程序文件 $PROGRAM_PATH"
    echo "[autostart] 请确认 build 已完成，或修改 PROGRAM_PATH。"
    exit 1
fi

# 3. 启动程序
start_program

# 4. 启动看门狗（在后台运行）
echo "[autostart] 启动看门狗 (间隔 ${WATCHDOG_INTERVAL}s)..."
watchdog &

# 5. 等待看门狗进程
wait
