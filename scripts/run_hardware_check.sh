#!/bin/bash
# Go2-X5 硬件检测测试脚本 (安全模式)
# 在Jetson开发板上运行（机器人身上），只检测硬件，不控制机器人

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
BUILD_DIR="$PROJECT_ROOT/cmake_build_refactor"

# 颜色输出
RED='\033[31m'
GREEN='\033[32m'
YELLOW='\033[33m'
NC='\033[0m'

log_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

print_banner() {
    echo ""
    echo "========================================"
    echo "  Go2-X5 硬件检测测试 (安全模式)"
    echo "  Jetson开发板 (机器人身上)"
    echo "========================================"
    echo ""
    echo -e "${YELLOW}⚠️  安全承诺：此测试只检测硬件，不控制机器人运动${NC}"
    echo ""
    echo -e "${YELLOW}  ✗ 不发送CAN控制帧${NC}"
    echo -e "${YELLOW}  ✗ 不发送DDS控制命令${NC}"
    echo -e "${YELLOW}  ✗ 不初始化真实硬件${NC}"
    echo ""
}

print_summary() {
    echo ""
    echo "========================================"
    if [ $FAILED -eq 0 ]; then
        echo -e "${GREEN}✓ 所有检测通过${NC}"
        echo "========================================"
        echo ""
        echo -e "${GREEN}硬件连接正常，可以准备上机控制${NC}"
        echo ""
        echo -e "${YELLOW}上机提示：${NC}"
        echo "1. 确认物理急停按钮可用"
        echo "2. 按 Ctrl+C 终止测试后，可以运行真实控制系统："
        echo "   ros2 launch rl_sar go2_x5_real_dual.launch.py \\"
        echo "     network_interface:=eth0 \\"
        echo "     arm_interface_name:=can0"
    else
        echo -e "${RED}✗ $FAILED 个检测失败${NC}"
        echo "========================================"
        echo ""
        echo -e "${RED}请修复上述问题后再尝试上机${NC}"
        echo ""
        echo "常见问题："
        echo "- CAN接口失败：sudo ip link set can0 up"
        echo "- SDK路径错误：export ARX5_SDK_ROOT=~/arx5-sdk"
    fi
    echo ""
}

# 检查构建目录
if [ ! -d "$BUILD_DIR" ]; then
    log_error "构建目录不存在: $BUILD_DIR"
    log_info "请先运行: cd ~/rl_ras_n && ./build.sh"
    exit 1
fi

# 检查可执行文件
if [ ! -f "$BUILD_DIR/bin/test_hardware_interfaces" ]; then
    log_error "测试未编译: $BUILD_DIR/bin/test_hardware_interfaces"
    log_info "请先运行: cd $BUILD_DIR && cmake .. && make test_hardware_interfaces"
    exit 1
fi

# 默认配置
CAN_INTERFACE="${CAN_INTERFACE:-can0}"
NET_INTERFACE="${NET_INTERFACE:-eth0}"
ARX_SDK_ROOT="${ARX5_SDK_ROOT:-}"
UNITREE_SDK2_ROOT="${UNITREE_SDK2_ROOT:-}"
MANIFEST_PATH="${MANIFEST_PATH:-deploy/go2_x5_real.yaml}"
POLICY_DIR="${POLICY_DIR:-.}"

# 解析参数
while [[ $# -gt 0 ]]; do
    case $1 in
        --can)
            CAN_INTERFACE="$2"
            shift 2
            ;;
        --net)
            NET_INTERFACE="$2"
            shift 2
            ;;
        --arx-sdk)
            ARX_SDK_ROOT="$2"
            shift 2
            ;;
        --unitree-sdk)
            UNITREE_SDK2_ROOT="$2"
            shift 2
            ;;
        --manifest)
            MANIFEST_PATH="$2"
            shift 2
            ;;
        --policy-dir)
            POLICY_DIR="$2"
            shift 2
            ;;
        --help|-h)
            echo "用法: $0 [选项]"
            echo ""
            echo "选项:"
            echo "  --can <接口>         CAN接口名 (默认: can0)"
            echo "  --net <接口>         网络接口名 (默认: eth0)"
            echo "  --arx-sdk <路径>     ARX SDK路径 (默认: \$ARX5_SDK_ROOT)"
            echo "  --unitree-sdk <路径> Unitree SDK路径 (默认: \$UNITREE_SDK2_ROOT)"
            echo "  --manifest <路径>    Deploy manifest路径"
            echo "  --policy-dir <路径>  策略文件目录"
            echo "  --help, -h           显示此帮助"
            echo ""
            echo "安全承诺："
            echo "  此测试只检测硬件接口，不会发送任何控制命令"
            echo "  机器人将保持完全静止状态"
            exit 0
            ;;
        *)
            log_error "未知参数: $1"
            echo "使用 --help 查看帮助"
            exit 1
            ;;
    esac
done

FAILED=0

# 运行硬件接口检测
print_banner
log_info "运行硬件接口检测..."

"$BUILD_DIR/bin/test_hardware_interfaces" \
    --can "$CAN_INTERFACE" \
    --net "$NET_INTERFACE" \
    --arx-sdk "$ARX_SDK_ROOT" \
    --unitree-sdk "$UNITREE_SDK2_ROOT" \
    --manifest "$MANIFEST_PATH" \
    --policy-dir "$POLICY_DIR" || FAILED=$((FAILED + 1))

print_summary $FAILED

exit $FAILED
