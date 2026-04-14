#!/bin/bash
# Go2-X5 启动前检查脚本
# 可在开发机离线运行，也可在 Jetson 上做真机前硬件探测

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
BUILD_DIR="$PROJECT_ROOT/cmake_build_refactor"
HARDWARE_BIN=""
MODE="offline"
SOFTWARE_TEST_REGEX="^(test_go2_x5_launch_defaults|test_go2_x5_deploy_manifest_loader|test_go2_x5_manifest_contract|test_go2_x5_runtime_validator|test_go2_x5_startup_safety_contract|test_go2_x5_runtime_smoke|test_go2_x5_arm_safety_guards|test_arm_adapter_contract)$"

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
    echo "  Go2-X5 启动前检查"
    echo "  Mode: $MODE"
    echo "========================================"
    echo ""
    echo -e "${YELLOW}⚠️  安全承诺：此检查只检测，不控制机器人运动${NC}"
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
        echo -e "${GREEN}✓ 启动前检查通过${NC}"
        echo "========================================"
        echo ""
        if [ "$MODE" = "offline" ]; then
            echo -e "${GREEN}离线静态检查通过${NC}"
            echo "仍建议在 Jetson 上启动双系统前再跑一次硬件模式："
            echo "  ./scripts/run_hardware_check.sh --mode hardware"
        else
            echo -e "${GREEN}软件与硬件检查都通过，可以准备启动双系统${NC}"
            echo ""
            echo -e "${YELLOW}上机提示：${NC}"
            echo "1. 确认物理急停按钮可用"
            echo "2. 启动前再次确认 can0 / eth0 / SDK 环境变量"
            echo "3. 再运行真实控制系统："
            echo "   ros2 launch rl_sar go2_x5_real_dual.launch.py \\"
            echo "     deploy_manifest_path:=~/rl_ras_n/deploy/go2_x5_real.yaml \\"
            echo "     network_interface:=eth0"
        fi
    else
        echo -e "${RED}✗ 启动前检查失败${NC}"
        echo "========================================"
        echo ""
        echo -e "${RED}请修复上述问题后再尝试上机${NC}"
        echo ""
        echo "常见问题："
        echo "- Manifest / model 检查失败：确认 deploy/go2_x5_real.yaml 和 policy.pt/policy.onnx"
        echo "- CAN接口失败：sudo ip link set can0 up"
        echo "- SDK路径错误：export ARX5_SDK_ROOT=~/arx5-sdk"
    fi
    echo ""
}

ensure_build_tree() {
    log_info "配置并构建启动前检查依赖..."
    cmake -S "$PROJECT_ROOT/src/rl_sar" -B "$BUILD_DIR" -DUSE_CMAKE=ON -DBUILD_TESTING=ON
    cmake --build "$BUILD_DIR" --target \
        test_go2_x5_launch_defaults \
        test_go2_x5_deploy_manifest_loader \
        test_go2_x5_manifest_contract \
        test_go2_x5_runtime_validator \
        test_go2_x5_startup_safety_contract \
        test_go2_x5_arm_safety_guards \
        test_go2_x5_runtime_smoke \
        test_arm_adapter_contract \
        test_hardware_interfaces
}

resolve_hardware_bin() {
    local candidates=(
        "$BUILD_DIR/bin/test_hardware_interfaces"
        "$BUILD_DIR/test_hardware_interfaces"
    )
    for candidate in "${candidates[@]}"; do
        if [ -x "$candidate" ]; then
            HARDWARE_BIN="$candidate"
            return 0
        fi
    done
    return 1
}

run_software_preflight() {
    log_info "运行软件侧启动前检查..."
    ctest --test-dir "$BUILD_DIR" --output-on-failure -R "$SOFTWARE_TEST_REGEX"
}

run_python_syntax_checks() {
    log_info "检查 launch 脚本语法..."
    python3 -m py_compile "$PROJECT_ROOT/src/rl_sar/launch/go2_x5_real_dual.launch.py"
}

run_runtime_preflight() {
    log_info "运行 runtime 资产与接口检查..."
    "$HARDWARE_BIN" \
        --mode "$MODE" \
        --can "$CAN_INTERFACE" \
        --net "$NET_INTERFACE" \
        --arx-sdk "$ARX_SDK_ROOT" \
        --arx-sdk-lib "$ARX_SDK_LIB_PATH" \
        --unitree-sdk "$UNITREE_SDK2_ROOT" \
        --manifest "$MANIFEST_PATH" \
        --policy-dir "$POLICY_DIR"
}

# 默认配置
CAN_INTERFACE="${CAN_INTERFACE:-can0}"
NET_INTERFACE="${NET_INTERFACE:-eth0}"
ARX_SDK_ROOT="${ARX5_SDK_ROOT:-}"
ARX_SDK_LIB_PATH="${ARX5_SDK_LIB_PATH:-}"
UNITREE_SDK2_ROOT="${UNITREE_SDK2_ROOT:-}"
MANIFEST_PATH="${MANIFEST_PATH:-$PROJECT_ROOT/deploy/go2_x5_real.yaml}"
POLICY_DIR="${POLICY_DIR:-$PROJECT_ROOT}"

# 解析参数
while [[ $# -gt 0 ]]; do
    case $1 in
        --mode)
            MODE="$2"
            shift 2
            ;;
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
        --arx-sdk-lib)
            ARX_SDK_LIB_PATH="$2"
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
            echo "  --mode <offline|hardware> 启动前检查模式 (默认: offline)"
            echo "  --can <接口>         CAN接口名 (默认: can0)"
            echo "  --net <接口>         网络接口名 (默认: eth0)"
            echo "  --arx-sdk <路径>     ARX SDK路径 (默认: \$ARX5_SDK_ROOT)"
            echo "  --arx-sdk-lib <路径> ARX SDK库目录或libhardware.so路径"
            echo "  --unitree-sdk <路径> Unitree SDK路径 (默认: \$UNITREE_SDK2_ROOT)"
            echo "  --manifest <路径>    Deploy manifest路径"
            echo "  --policy-dir <路径>  策略文件目录"
            echo "  --help, -h           显示此帮助"
            echo ""
            echo "安全承诺："
            echo "  此检查只检测运行时资产和接口，不会发送任何控制命令"
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

if [[ "$MODE" != "offline" && "$MODE" != "hardware" ]]; then
    log_error "无效模式: $MODE"
    echo "允许值: offline, hardware"
    exit 1
fi

FAILED=0

print_banner
ensure_build_tree || FAILED=$((FAILED + 1))

if [ $FAILED -eq 0 ]; then
    resolve_hardware_bin || FAILED=$((FAILED + 1))
fi

if [ $FAILED -eq 0 ]; then
    run_software_preflight || FAILED=$((FAILED + 1))
fi

if [ $FAILED -eq 0 ]; then
    run_python_syntax_checks || FAILED=$((FAILED + 1))
fi

if [ $FAILED -eq 0 ]; then
    run_runtime_preflight || FAILED=$((FAILED + 1))
fi

print_summary $FAILED

exit $FAILED
