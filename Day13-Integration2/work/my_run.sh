#!/bin/bash
# ================================================================
# my_run.sh —— Day13 桌面双击运行脚本
#
# #### Task 13-5b: 编写桌面运行脚本 ##############################
#
# 使用方法：
#   1. chmod +x my_run.sh
#   2. ./my_run.sh （终端运行）
#   3. 或创建 .desktop 文件实现桌面双击启动
#
# 与 my_autostart.sh 的区别：
#   - my_autostart.sh: 开机自启动 + 看门狗守护
#   - my_run.sh:       手动启动（方便调试和演示）
# ================================================================

# ============================================================
# ★ 配置区域 ★
# ============================================================
PROGRAM_PATH="/home/ad/Training-26-Lecture1/Horizon_Vision_Tutorial_26/Day13-Integration2/work/build/my_full_pipeline"
CONFIG_DIR="/home/ad/Training-26-Lecture1/Horizon_Vision_Tutorial_26/Day13-Integration2/work/build"

# ============================================================
# ★ 主流程 ★
# ============================================================

echo "========================================"
echo "  Horizon Vision Pipeline"
echo "  手动启动模式"
echo "========================================"

# 1. 串口赋权
echo "[run] 为串口赋权..."
for dev in /dev/ttyUSB*; do
    if [ -e "$dev" ]; then
        sudo chmod 666 "$dev"
        echo "[run] 已赋权 $dev"
    fi
done

# 2. 启动程序
if [ ! -f "$PROGRAM_PATH" ]; then
    echo "[run] 错误: 找不到程序 $PROGRAM_PATH"
    echo "[run] 请先编译: cd build && cmake .. && make"
    exit 1
fi

cd "$CONFIG_DIR" || exit 1
echo "[run] 启动 Pipeline..."
$PROGRAM_PATH

echo "[run] Pipeline 已退出。"
