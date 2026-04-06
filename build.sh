#!/bin/bash
set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
source "${SCRIPT_DIR}/scripts/common.sh"

: "${NO_GITHUB_HTTPS:=true}"

ensure_package_symlink() {
    local pkg_dir="${SCRIPT_DIR}/src/rl_sar"
    if [ -f "${pkg_dir}/package.ros2.xml" ] && [ ! -e "${pkg_dir}/package.xml" ]; then
        ln -s package.ros2.xml "${pkg_dir}/package.xml"
    fi
}

setup_inference_runtime() {
    print_header "[Setting up Inference Runtime]"
    local download_script="${SCRIPT_DIR}/scripts/download_inference_runtime.sh"

    if [ ! -f "$download_script" ]; then
        print_warning "Download script not found: $download_script"
        return 0
    fi

    local download_target="all"
    if [ -f /etc/nv_tegra_release ] || [ "${NO_GITHUB_HTTPS}" = "true" ]; then
        download_target="libtorch"
        print_info "NO_GITHUB_HTTPS mode enabled: skip ONNX GitHub HTTPS download."
    fi

    bash "$download_script" "$download_target"
    print_success "Inference runtime setup completed!"
}

cmake_sdk_args() {
    local args=()
    if [ -n "${UNITREE_SDK2_ROOT:-}" ]; then
        args+=("-DUNITREE_SDK2_ROOT=${UNITREE_SDK2_ROOT}")
    fi
    printf '%s\n' "${args[@]}"
}

run_cmake_build() {
    print_header "[Running CMake Build]"
    local extra_args=()
    while IFS= read -r arg; do
        [ -n "$arg" ] && extra_args+=("$arg")
    done < <(cmake_sdk_args)

    cmake -S src/rl_sar -B cmake_build -DUSE_CMAKE=ON "${extra_args[@]}"
    cmake --build cmake_build -j"$(nproc 2>/dev/null || echo 4)"
    print_success "CMake build completed!"
}

run_ros_build() {
    local packages=("$@")
    local colcon_cmd=(colcon build --merge-install --symlink-install)
    local cmake_args=()

    ensure_package_symlink

    if [[ "$ROS_DISTRO" == "foxy" ]] && [ -x /usr/bin/python3 ]; then
        cmake_args+=("-DPython3_EXECUTABLE=/usr/bin/python3")
        cmake_args+=("-DPython3_ROOT_DIR=/usr")
        cmake_args+=("-DPython3_FIND_STRATEGY=LOCATION")
    fi

    while IFS= read -r arg; do
        [ -n "$arg" ] && cmake_args+=("$arg")
    done < <(cmake_sdk_args)

    if [ ${#packages[@]} -gt 0 ]; then
        colcon_cmd+=(--packages-select "${packages[@]}")
    else
        colcon_cmd+=(--packages-select rl_sar)
    fi

    if [ ${#cmake_args[@]} -gt 0 ]; then
        colcon_cmd+=(--cmake-args "${cmake_args[@]}")
    fi

    "${colcon_cmd[@]}"
    print_success "ROS build completed!"
}

clean_workspace() {
    print_header "[Cleaning Workspace]"
    rm -rf build/ install/ log/ cmake_build/ cmake_build_* devel/ logs/ .catkin_tools/
    print_success "Clean completed!"
}

show_usage() {
    print_header "[Build System Usage]"
    echo "Usage: $0 [OPTIONS] [PACKAGE_NAMES...]"
    echo ""
    echo "Options:"
    echo "  -c, --clean      Clean build artifacts"
    echo "  -m, --cmake      Standalone CMake build"
    echo "  -h, --help       Show this help message"
    echo ""
    echo "Examples:"
    echo "  $0"
    echo "  $0 rl_sar"
    echo "  $0 --cmake"
    echo "  $0 --clean"
}

main() {
    local packages=()
    local clean_mode=false
    local cmake_mode=false

    while [[ $# -gt 0 ]]; do
        case $1 in
            -c|--clean) clean_mode=true; shift ;;
            -m|--cmake) cmake_mode=true; shift ;;
            -h|--help) show_usage; exit 0 ;;
            --) shift; packages+=("$@"); break ;;
            -*) print_error "Unknown option: $1"; show_usage; exit 1 ;;
            *) packages+=("$1"); shift ;;
        esac
    done

    if [ "$clean_mode" = true ]; then
        clean_workspace
        exit 0
    fi

    setup_inference_runtime

    if [ "$cmake_mode" = true ]; then
        run_cmake_build
        exit 0
    fi

    if [ -z "${ROS_DISTRO:-}" ]; then
        print_error "ROS environment not detected. Please source /opt/ros/foxy/setup.bash first."
        exit 1
    fi
    if [ "$ROS_DISTRO" != "foxy" ]; then
        print_error "This workspace only supports ROS2 Foxy. Current ROS_DISTRO=$ROS_DISTRO"
        exit 1
    fi

    run_ros_build "${packages[@]}"
}

main "$@"
