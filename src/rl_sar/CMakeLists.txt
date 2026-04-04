cmake_minimum_required(VERSION 3.10)
project(rl_sar VERSION 4.0.0 LANGUAGES CXX)

# Set C++ standard globally
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# System environment detection
set(SYSTEM_TYPE ${CMAKE_SYSTEM_NAME})

set(USE_CMAKE OFF CACHE BOOL "Use Cmake build system")
if(USE_CMAKE)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
endif()

if(NOT USE_CMAKE)
    if($ENV{ROS_DISTRO} MATCHES "foxy")
        add_compile_definitions(USE_ROS)
        add_compile_definitions(USE_ROS2)
    else()
        message(FATAL_ERROR "rl_ras_n supports ROS2 Foxy only. Current ROS_DISTRO=$ENV{ROS_DISTRO}")
    endif()
else()
    message(WARNING "ROS_DISTRO not set, assuming non-ROS mode")
    add_compile_definitions(USE_CMAKE)
endif()

# MuJoCo simulator option
set(USE_MUJOCO OFF CACHE BOOL "Build with MuJoCo simulator support")

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()
string(TOUPPER "${CMAKE_BUILD_TYPE}" UPPER_BUILD_TYPE)
if(UPPER_BUILD_TYPE STREQUAL "DEBUG")
    add_compile_options(-g)
endif()

# Get project root directory
get_filename_component(PROJECT_ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)

add_definitions(-DCMAKE_CURRENT_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
add_definitions(-DPOLICY_DIR="${PROJECT_ROOT_DIR}/policy")
add_definitions(-DBOOST_BIND_GLOBAL_PLACEHOLDERS)

if(NOT USE_CMAKE)
    if($ENV{ROS_DISTRO} MATCHES "foxy")
        find_package(ament_cmake REQUIRED)
        find_package(joint_state_broadcaster REQUIRED)
        find_package(robot_state_publisher REQUIRED)
        find_package(rclcpp REQUIRED)
        find_package(gazebo_ros REQUIRED)
        find_package(std_msgs REQUIRED)
        find_package(sensor_msgs REQUIRED)
        find_package(robot_msgs REQUIRED)
        find_package(robot_joint_controller REQUIRED)
        find_package(rclpy REQUIRED)
        find_package(gazebo_msgs REQUIRED)
        find_package(std_srvs REQUIRED)
        find_package(geometry_msgs REQUIRED)
        ament_package()
    else()
        message(FATAL_ERROR "rl_ras_n supports ROS2 Foxy only. Current ROS_DISTRO=$ENV{ROS_DISTRO}")
    endif()
    add_compile_options(${GAZEBO_CXX_FLAGS})
    find_package(gazebo REQUIRED)
endif()

# Inference Runtime

set(INFERENCE_RUNTIME_DIR "${PROJECT_ROOT_DIR}/library/inference_runtime")

# ONNX Runtime Setup (default and only required backend for deployment)
set(USE_ONNX "OFF")
set(ONNX_RUNTIME_LIB "")
set(ONNX_RUNTIME_DIR "${INFERENCE_RUNTIME_DIR}/onnxruntime")

# Detect Jetson platform (for informational purposes only)
set(IS_JETSON FALSE)
if(EXISTS "/etc/nv_tegra_release")
    set(IS_JETSON TRUE)
    message(STATUS "Jetson platform detected - ARM64 ONNX Runtime will be used")
endif()

if(EXISTS "${ONNX_RUNTIME_DIR}/include" AND EXISTS "${ONNX_RUNTIME_DIR}/lib")
    set(USE_ONNX "ON")
    add_compile_definitions(USE_ONNX)

    message(STATUS "Found ONNX Runtime at: ${ONNX_RUNTIME_DIR}")

    # Read ONNX Runtime version
    if(EXISTS "${ONNX_RUNTIME_DIR}/VERSION_NUMBER")
        file(READ "${ONNX_RUNTIME_DIR}/VERSION_NUMBER" ONNX_RUNTIME_VERSION)
        string(STRIP "${ONNX_RUNTIME_VERSION}" ONNX_RUNTIME_VERSION)
    endif()

    include_directories("${ONNX_RUNTIME_DIR}/include")

    # Detect library extension
    if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
        set(LIB_EXT "so")
    elseif(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
        set(LIB_EXT "dylib")
    elseif(CMAKE_SYSTEM_NAME STREQUAL "Windows")
        set(LIB_EXT "lib")
    endif()

    # Find library
    if(CMAKE_SYSTEM_NAME STREQUAL "Windows")
        set(LIB_NAMES onnxruntime.lib)
    else()
        set(LIB_NAMES libonnxruntime.${LIB_EXT} onnxruntime.${LIB_EXT})
    endif()

    find_library(ONNX_RUNTIME_LIB_PATH
        NAMES ${LIB_NAMES}
        PATHS "${ONNX_RUNTIME_DIR}/lib"
        NO_DEFAULT_PATH
    )

    if(ONNX_RUNTIME_LIB_PATH)
        if(CMAKE_SYSTEM_NAME STREQUAL "Windows")
            add_library(onnxruntime_prebuilt STATIC IMPORTED GLOBAL)
        else()
            add_library(onnxruntime_prebuilt SHARED IMPORTED GLOBAL)
        endif()

        set_target_properties(onnxruntime_prebuilt PROPERTIES
            IMPORTED_LOCATION "${ONNX_RUNTIME_LIB_PATH}"
            INTERFACE_INCLUDE_DIRECTORIES "${ONNX_RUNTIME_DIR}/include"
        )

        set(ONNX_RUNTIME_LIB onnxruntime_prebuilt)
        message(STATUS "ONNX Runtime library: ${ONNX_RUNTIME_LIB_PATH}")
    else()
        message(WARNING "ONNX Runtime library not found")
        set(USE_ONNX "OFF")
    endif()
else()
    message(FATAL_ERROR "ONNX Runtime is required for deployment. Please run: bash scripts/download_inference_runtime.sh onnx")
endif()

# LibTorch Setup (optional - only needed for training/research, NOT for deployment)
option(ENABLE_TORCH_BACKEND "Enable PyTorch/LibTorch backend (not required for ONNX deployment)" OFF)
set(USE_TORCH "OFF")
set(TORCH_LIBRARIES "")
set(LIBTORCH_DIR "${INFERENCE_RUNTIME_DIR}/libtorch")

if(ENABLE_TORCH_BACKEND AND EXISTS "${LIBTORCH_DIR}/include" AND EXISTS "${LIBTORCH_DIR}/lib")
    set(USE_TORCH "ON")
    add_compile_definitions(USE_TORCH)

    message(STATUS "Found LibTorch at: ${LIBTORCH_DIR}")

    # For Jetson with old CMake: set common Jetson CUDA architectures to avoid CUDA17 requirement
    if(CMAKE_VERSION VERSION_LESS "3.18")
        # Common Jetson architectures: 5.3(Nano), 6.2(TX2), 7.2(Xavier), 8.7(Orin)
        set(TORCH_CUDA_ARCH_LIST "5.3;6.2;7.2;8.7" CACHE STRING "CUDA architectures")
        set(ENV{TORCH_CUDA_ARCH_LIST} "5.3;6.2;7.2;8.7")
        message(STATUS "CMake ${CMAKE_VERSION} detected - using common Jetson CUDA architectures")
    endif()

    set(CMAKE_PREFIX_PATH "${LIBTORCH_DIR};${CMAKE_PREFIX_PATH}")
    find_package(Torch QUIET)

    if(Torch_FOUND)
        message(STATUS "LibTorch version: ${Torch_VERSION}")
        message(STATUS "Torch libraries: ${TORCH_LIBRARIES}")
    else()
        set(USE_TORCH "OFF")
        message(WARNING "LibTorch found but find_package(Torch) failed")
    endif()
else()
    message(STATUS "LibTorch not enabled or not found (optional - only needed for Torch model support)")
    message(STATUS "  To enable: -DENABLE_TORCH_BACKEND=ON")
    message(STATUS "  To install: bash scripts/download_inference_runtime.sh libtorch")
endif()

# RPATH Configuration for inference libraries
if(USE_TORCH OR USE_ONNX)
    set(CMAKE_SKIP_BUILD_RPATH FALSE)
    set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
    set(CMAKE_INSTALL_RPATH_USE_LINK_PATH FALSE)

    set(PREBUILT_LIB_PATHS "")
    if(USE_TORCH)
        list(APPEND PREBUILT_LIB_PATHS "${LIBTORCH_DIR}/lib")
    endif()
    if(USE_ONNX)
        list(APPEND PREBUILT_LIB_PATHS "${ONNX_RUNTIME_DIR}/lib")
    endif()
    # OpenMP path will be added later after it's found on macOS
    set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_RPATH};${PREBUILT_LIB_PATHS}")

    if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
        set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--disable-new-dtags")
        set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--disable-new-dtags")
    endif()

    message(STATUS "Configured RPATH:")
    foreach(path ${PREBUILT_LIB_PATHS})
        message(STATUS "  - ${path}")
    endforeach()
endif()

find_package(TBB REQUIRED)
find_package(Threads REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)

# Add Python library path to RPATH
if(Python3_LIBRARIES)
    get_filename_component(PYTHON_LIB_DIR "${Python3_LIBRARIES}" DIRECTORY)
    set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_RPATH};${PYTHON_LIB_DIR}")
    message(STATUS "Added Python library path to RPATH: ${PYTHON_LIB_DIR}")
endif()

if(SYSTEM_TYPE STREQUAL "Linux")
    set(YAML_CPP_LIBRARIES yaml-cpp)
link_directories(/usr/local/lib)
include_directories(${YAML_CPP_INCLUDE_DIR})
elseif(SYSTEM_TYPE STREQUAL "Darwin")
    find_library(YAML_CPP_LIB yaml-cpp REQUIRED)
    set(YAML_CPP_LIBRARIES ${YAML_CPP_LIB})
    link_directories(/opt/homebrew/lib)
    if(YAML_CPP_INCLUDE_DIR)
        include_directories(${YAML_CPP_INCLUDE_DIR})
    endif()

    # Find OpenMP (required by LibTorch on macOS)
    find_library(OMP_LIB NAMES omp PATHS /opt/homebrew/lib /opt/homebrew/opt/libomp/lib)
    if(OMP_LIB)
        message(STATUS "Found OpenMP library: ${OMP_LIB}")
        set(OPENMP_LIBRARIES ${OMP_LIB})
        # Add OpenMP path to RPATH (for LibTorch dependency)
        get_filename_component(OMP_LIB_DIR "${OPENMP_LIBRARIES}" DIRECTORY)
        set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_RPATH};${OMP_LIB_DIR}")
        message(STATUS "Added OpenMP to RPATH: ${OMP_LIB_DIR}")
    else()
        message(WARNING "OpenMP not found. Please install: brew install libomp")
    endif()
endif()

if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64|ARM64")
    message(STATUS "Current system architecture: ${CMAKE_SYSTEM_PROCESSOR}")
    set(ARCH_DIR "aarch64")
    set(UNITREE_LIB "libunitree_legged_sdk_arm64")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|amd64|AMD64")
message(STATUS "Current system architecture: ${CMAKE_SYSTEM_PROCESSOR}")
    set(ARCH_DIR "x86_64")
    set(UNITREE_LIB "libunitree_legged_sdk_amd64")
else()
    message(FATAL_ERROR "Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

# unitree_legged_sdk (removed - A1 support removed, only go2-x5 supported)

# unitree_sdk2
set(UNITREE_SDK2_AVAILABLE OFF)
set(UNITREE_SDK2_THIRDPARTY_INCLUDE "")
set(UNITREE_SDK2_THIRDPARTY_DDSCXX_INCLUDE "")
set(UNITREE_SDK2_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/library/thirdparty/robot_sdk/unitree/unitree_sdk2" CACHE PATH
    "Path to a Unitree SDK2 checkout")
if(SYSTEM_TYPE STREQUAL "Linux")
    if(EXISTS "${UNITREE_SDK2_ROOT}/CMakeLists.txt")
        set(UNITREE_SDK2_AVAILABLE ON)
        set(UNITREE_SDK2_THIRDPARTY_INCLUDE
            "${UNITREE_SDK2_ROOT}/thirdparty/include")
        set(UNITREE_SDK2_THIRDPARTY_DDSCXX_INCLUDE
            "${UNITREE_SDK2_ROOT}/thirdparty/include/ddscxx")
        set(BUILD_EXAMPLES OFF CACHE BOOL "Disable examples build for unitree_sdk2" FORCE)
        add_subdirectory("${UNITREE_SDK2_ROOT}" "${CMAKE_BINARY_DIR}/unitree_sdk2")
    else()
        message(WARNING "unitree_sdk2 not found at ${UNITREE_SDK2_ROOT}; skipping rl_real_go2_x5 target")
    endif()
endif()

# arx_x5_sdk (optional, for go2_x5 split arm channel)
set(ARX_X5_AVAILABLE OFF)
option(USE_ARX_X5_SDK "Enable ARX X5 SDK integration for go2_x5 arm channel" OFF)
set(ARX_X5_SDK_INCLUDE_DIR "" CACHE PATH "ARX X5 SDK include directory")
set(ARX_X5_SDK_LIBRARY "" CACHE FILEPATH "ARX X5 SDK shared library path")
if(USE_ARX_X5_SDK)
    if(EXISTS "${ARX_X5_SDK_INCLUDE_DIR}" AND EXISTS "${ARX_X5_SDK_LIBRARY}")
        set(ARX_X5_AVAILABLE ON)
        add_library(arx_x5_sdk SHARED IMPORTED)
        set_target_properties(arx_x5_sdk PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES "${ARX_X5_SDK_INCLUDE_DIR}"
            IMPORTED_LOCATION "${ARX_X5_SDK_LIBRARY}"
        )
        message(STATUS "ARX X5 SDK enabled: ${ARX_X5_SDK_LIBRARY}")
    else()
        message(WARNING "USE_ARX_X5_SDK=ON but ARX_X5_SDK_INCLUDE_DIR / ARX_X5_SDK_LIBRARY is invalid.")
    endif()
endif()

# l4w4_sdk (removed - L4W4 support removed, only go2-x5 supported)

# Lite3_MotionSDK (removed - Lite3 support removed, only go2-x5 supported)
# Retroid Gamepad (removed - only used by Lite3)

if(NOT USE_CMAKE)
    if(FALSE)
        include_directories(${catkin_INCLUDE_DIRS})
    endif()
endif()

include_directories(
    include
    include/rl_sar
    include/rl_sar/core
    include/rl_sar/go2x5
    include/rl_sar/go2x5/arm
    include/rl_sar/go2x5/control
    include/rl_sar/go2x5/config
    include/rl_sar/go2x5/safety
    include/rl_sar/go2x5/comm
    include/rl_sar/go2x5/state
    include/rl_sar/diagnostics
    include/rl_sar/logger
    include/rl_sar/adapters
    include/rl_sar/protocol
    include/rl_sar/runtime
    library/core/matplotlibcpp
    library/core/observation_buffer
    library/core/rl_sdk
    library/core/loop
    library/core/fsm
    library/core/vector_math
    library/core/motion_loader
    library/core/inference_runtime
    library/core/logger
    fsm_robot
)

# library for inference runtime
add_library(inference_runtime library/core/inference_runtime/inference_runtime.cpp)
if(USE_TORCH)
    target_link_libraries(inference_runtime PUBLIC ${TORCH_LIBRARIES})
    target_compile_definitions(inference_runtime PRIVATE USE_TORCH)
    # Link OpenMP on macOS (required by LibTorch)
    if(SYSTEM_TYPE STREQUAL "Darwin" AND OPENMP_LIBRARIES)
        target_link_libraries(inference_runtime PUBLIC ${OPENMP_LIBRARIES})
    endif()
endif()
if(USE_ONNX)
    target_link_libraries(inference_runtime PUBLIC ${ONNX_RUNTIME_LIB})
    target_compile_definitions(inference_runtime PRIVATE USE_ONNX)
endif()
if(NOT USE_CMAKE)
    if($ENV{ROS_DISTRO} MATCHES "foxy")
        install(TARGETS inference_runtime DESTINATION lib/${PROJECT_NAME})
    endif()
endif()

# library for motion loader
add_library(motion_loader library/core/motion_loader/motion_loader.cpp)

# library for rl sdk
add_library(rl_sdk library/core/rl_sdk/rl_sdk.cpp src/rl_sar/go2x5/control/go2_x5_operator_control.cpp)
target_link_libraries(rl_sdk PUBLIC
    motion_loader
    inference_runtime
    Python3::Python
    Python3::Module
    TBB::tbb
)
find_package(Python3 COMPONENTS NumPy)
if(Python3_NumPy_FOUND)
    target_link_libraries(rl_sdk PUBLIC Python3::NumPy)
else()
    target_compile_definitions(rl_sdk PUBLIC WITHOUT_NUMPY)
endif()
if(NOT USE_CMAKE)
    if($ENV{ROS_DISTRO} MATCHES "foxy")
        install(TARGETS rl_sdk DESTINATION lib/${PROJECT_NAME})
    endif()
endif()

# library for observation buffer
add_library(observation_buffer library/core/observation_buffer/observation_buffer.cpp)
if(NOT USE_CMAKE)
    if($ENV{ROS_DISTRO} MATCHES "foxy")
        install(TARGETS observation_buffer DESTINATION lib/${PROJECT_NAME})
    endif()
endif()

# library for event logger
add_library(event_logger src/rl_sar/logger/event_logger.cpp)
target_link_libraries(event_logger PUBLIC Threads::Threads)
if(NOT USE_CMAKE)
    if($ENV{ROS_DISTRO} MATCHES "foxy")
        install(TARGETS event_logger DESTINATION lib/${PROJECT_NAME})
    endif()
endif()

# library for unitree adapter (isolates DDS from main logic)
if(SYSTEM_TYPE STREQUAL "Linux" AND UNITREE_SDK2_AVAILABLE)
    add_library(unitree_adapter src/rl_sar/adapters/unitree/unitree_adapter.cpp)
    target_include_directories(unitree_adapter BEFORE PRIVATE
        ${UNITREE_SDK2_THIRDPARTY_INCLUDE}
        ${UNITREE_SDK2_THIRDPARTY_DDSCXX_INCLUDE}
    )
    target_link_libraries(unitree_adapter
        unitree_sdk2
    )
    set_target_properties(unitree_adapter PROPERTIES
        INSTALL_RPATH "${CMAKE_INSTALL_RPATH};${UNITREE_SDK2_ROOT}/thirdparty/lib/${ARCH_DIR}"
    )
    if(NOT USE_CMAKE)
        if($ENV{ROS_DISTRO} MATCHES "foxy")
            install(TARGETS unitree_adapter DESTINATION lib/${PROJECT_NAME})
        endif()
    endif()
endif()

# library for arx adapter (converges ARX SDK integration)
add_library(arx_adapter src/rl_sar/adapters/arx/arx_adapter.cpp)
target_link_libraries(arx_adapter PUBLIC Threads::Threads ${CMAKE_DL_LIBS})
if(NOT USE_CMAKE)
    if($ENV{ROS_DISTRO} MATCHES "foxy")
        install(TARGETS arx_adapter DESTINATION lib/${PROJECT_NAME})
    endif()
endif()

# executable for rl sim
if(NOT USE_CMAKE)
    add_executable(rl_sim src/rl_sar/core/rl_sim.cpp)
    target_link_libraries(rl_sim
        rl_sdk
        observation_buffer
        ${YAML_CPP_LIBRARIES}
        Threads::Threads
    )
    if(FALSE)
        target_link_libraries(rl_sim ${catkin_LIBRARIES})
    elseif($ENV{ROS_DISTRO} MATCHES "foxy")
        ament_target_dependencies(rl_sim
            joint_state_broadcaster
            robot_state_publisher
            rclcpp
            gazebo_ros
            std_msgs
            sensor_msgs
            robot_msgs
            robot_joint_controller
            rclpy
            gazebo_msgs
            std_srvs
            geometry_msgs
        )
        install(TARGETS rl_sim DESTINATION lib/${PROJECT_NAME})
    endif()
endif()

# executable for rl real a1 (removed - only go2-x5 supported)

# executable for rl real lite3 (removed - only go2-x5 supported)

# executable for rl real go2 (removed - only go2-x5 supported)

# executable for rl real go2_x5
if(SYSTEM_TYPE STREQUAL "Linux" AND UNITREE_SDK2_AVAILABLE)
    add_executable(rl_real_go2_x5
        # Core
        src/rl_sar/core/rl_real_go2_x5.cpp
        # Arm
        src/rl_sar/go2x5/arm/go2_x5_arm_runtime.cpp
        src/rl_sar/go2x5/arm/go2_x5_arm_bridge_runtime.cpp
        src/rl_sar/go2x5/arm/go2_x5_arm_output_guard.cpp
        src/rl_sar/go2x5/arm/go2_x5_arm_transport.cpp
        src/rl_sar/go2x5/arm/go2_x5_arm_controller.cpp
        src/rl_sar/go2x5/arm/go2_x5_arm_bridge_transport.cpp
        # Safety
        src/rl_sar/go2x5/safety/go2_x5_safety_guard.cpp
        src/rl_sar/go2x5/safety/go2_x5_safe_shutdown.cpp
        src/rl_sar/go2x5/safety/go2_x5_clip_validation.cpp
        src/rl_sar/go2x5/safety/go2_x5_limits.cpp
        # Control
        src/rl_sar/go2x5/control/go2_x5_operator_control.cpp
        src/rl_sar/go2x5/control/go2_x5_keyboard.cpp
        # Adapters
        src/rl_sar/adapters/unitree/unitree_adapter.cpp
        # Comm
        src/rl_sar/go2x5/comm/go2_x5_ipc.cpp
        src/rl_sar/go2x5/comm/go2_x5_ros.cpp
        src/rl_sar/go2x5/comm/go2_x5_unitree_comm.cpp
        # State
        src/rl_sar/go2x5/state/go2_x5_state_manager.cpp
        src/rl_sar/go2x5/state/go2_x5_state_io.cpp
        src/rl_sar/go2x5/state/go2_x5_state_management.cpp
        src/rl_sar/go2x5/state/go2_x5_utility.cpp
        # Runtime
        src/rl_sar/protocol/go2_x5_protocol.cpp
        src/rl_sar/runtime/supervisor/go2_x5_supervisor.cpp
        # Diagnostics
        src/rl_sar/diagnostics/diagnostics_publisher.cpp
        src/rl_sar/diagnostics/drift_recorder.cpp
        # Config
        library/core/config/config_loader.cpp
        library/core/config/deploy_manifest_loader.cpp
        library/core/config/deploy_manifest_runtime.cpp
    )
    target_include_directories(rl_real_go2_x5 BEFORE PRIVATE
        ${UNITREE_SDK2_THIRDPARTY_INCLUDE}
        ${UNITREE_SDK2_THIRDPARTY_DDSCXX_INCLUDE}
        ${CMAKE_CURRENT_SOURCE_DIR}
    )
    target_link_libraries(rl_real_go2_x5
        unitree_sdk2
        rl_sdk
        observation_buffer
        event_logger
        ${YAML_CPP_LIBRARIES}
    )
    # Add unitree_sdk2 library path to RPATH
    set_target_properties(rl_real_go2_x5 PROPERTIES
        INSTALL_RPATH "${CMAKE_INSTALL_RPATH};${UNITREE_SDK2_ROOT}/thirdparty/lib/${ARCH_DIR}"
    )
    if(NOT USE_CMAKE)
        if(FALSE)
            target_link_libraries(rl_real_go2_x5 ${catkin_LIBRARIES})
        elseif($ENV{ROS_DISTRO} MATCHES "foxy")
            ament_target_dependencies(rl_real_go2_x5
                rclcpp
                geometry_msgs
                std_msgs
            )
            install(TARGETS rl_real_go2_x5 DESTINATION lib/${PROJECT_NAME})
        endif()
    endif()
    if(ARX_X5_AVAILABLE)
        target_link_libraries(rl_real_go2_x5 arx_x5_sdk)
        target_compile_definitions(rl_real_go2_x5 PRIVATE USE_ARX_X5_SDK)
    endif()
endif()

# executable for rl real g1 (removed - only go2-x5 supported)

# executable for rl real l4w4 (removed - only go2-x5 supported)

# executable for actuator net
if(NOT USE_CMAKE)
    if(FALSE)
        catkin_install_python(PROGRAMS
            scripts/actuator_net.py
            scripts/arx_x5_bridge.py
            DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
    elseif($ENV{ROS_DISTRO} MATCHES "foxy")
        install(PROGRAMS
            scripts/actuator_net.py
            scripts/arx_x5_bridge.py
            DESTINATION lib/${PROJECT_NAME}
        )
    endif()
endif()

if(NOT USE_CMAKE)
    if($ENV{ROS_DISTRO} MATCHES "foxy")
        install(
            DIRECTORY launch worlds ${PROJECT_ROOT_DIR}/policy
            DESTINATION share/${PROJECT_NAME}/${dir}
        )
    endif()
endif()

# ========================
# mujoco
# ========================
if(USE_MUJOCO)
    add_compile_definitions(USE_MUJOCO)
    set(MUJOCO_DIR "${PROJECT_ROOT_DIR}/library/mujoco")

    # Check MuJoCo installation (different paths for different platforms)
    if(SYSTEM_TYPE STREQUAL "Darwin")
        # macOS uses framework structure
        if(NOT EXISTS "${MUJOCO_DIR}/Headers/mujoco.h")
            message(FATAL_ERROR "MuJoCo not found. Please run: bash scripts/download_mujoco.sh")
        endif()

        # Create compatible include structure in build directory for macOS
        # This allows <mujoco/mujoco.h> to work on macOS where headers are in Headers/ directory
        set(MUJOCO_COMPAT_INCLUDE_DIR "${CMAKE_BINARY_DIR}/mujoco_include")
        file(MAKE_DIRECTORY "${MUJOCO_COMPAT_INCLUDE_DIR}/mujoco")
        file(GLOB MUJOCO_HEADERS "${MUJOCO_DIR}/Headers/*.h")
        file(COPY ${MUJOCO_HEADERS} DESTINATION "${MUJOCO_COMPAT_INCLUDE_DIR}/mujoco")

        set(MUJOCO_INCLUDE_DIR "${MUJOCO_COMPAT_INCLUDE_DIR}")
        set(MUJOCO_LIB_DIR "${MUJOCO_DIR}/Versions/Current")
    else()
        # Linux/Windows use standard structure
        if(NOT EXISTS "${MUJOCO_DIR}/include/mujoco/mujoco.h")
            message(FATAL_ERROR "MuJoCo not found. Please run: bash scripts/download_mujoco.sh")
        endif()
        set(MUJOCO_INCLUDE_DIR "${MUJOCO_DIR}/include")
        set(MUJOCO_LIB_DIR "${MUJOCO_DIR}/lib")
    endif()

    # Add MuJoCo library path to global RPATH
    set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_RPATH};${MUJOCO_LIB_DIR}")
    message(STATUS "Added MuJoCo to RPATH: ${MUJOCO_LIB_DIR}")

    # Find GLFW
    find_package(glfw3 REQUIRED)

    # Platform-specific library names
    if(SYSTEM_TYPE STREQUAL "Linux")
        set(MUJOCO_LIB_NAME "libmujoco.so.3.2.7")
    elseif(SYSTEM_TYPE STREQUAL "Darwin")
        set(MUJOCO_LIB_NAME "libmujoco.3.2.7.dylib")
    elseif(SYSTEM_TYPE STREQUAL "Windows")
        set(MUJOCO_LIB_NAME "mujoco.dll")
    else()
        message(FATAL_ERROR "Unsupported platform: ${SYSTEM_TYPE}")
    endif()

    # Import MuJoCo library with RPATH
    add_library(mujoco_prebuilt SHARED IMPORTED)
    set_target_properties(mujoco_prebuilt PROPERTIES
        IMPORTED_LOCATION "${MUJOCO_LIB_DIR}/${MUJOCO_LIB_NAME}"
        INTERFACE_INCLUDE_DIRECTORIES "${MUJOCO_INCLUDE_DIR}"
        IMPORTED_NO_SONAME TRUE
    )

    # Include directories
    include_directories(
        ${MUJOCO_INCLUDE_DIR}
        library/thirdparty/joystick
    )

    # Source files
    file(GLOB MUJOCO_SIMULATE_SRC library/thirdparty/mujoco_simulate/*.cc)
    set(JOYSTICK_SRC library/thirdparty/joystick/joystick.cc)

    # On macOS, also include Objective-C++ files for CoreVideo support
    if(SYSTEM_TYPE STREQUAL "Darwin")
        file(GLOB MUJOCO_SIMULATE_MM library/thirdparty/mujoco_simulate/*.mm)
        list(APPEND MUJOCO_SIMULATE_SRC ${MUJOCO_SIMULATE_MM})
    endif()

    # Build executable
    add_executable(rl_sim_mujoco src/rl_sar/core/rl_sim_mujoco.cpp ${MUJOCO_SIMULATE_SRC} ${JOYSTICK_SRC})

    # Suppress deprecation warnings for macOS 15.0 API changes in MuJoCo Simulate
    if(SYSTEM_TYPE STREQUAL "Darwin")
        set_source_files_properties(
            ${MUJOCO_SIMULATE_MM}
            PROPERTIES COMPILE_FLAGS "-Wno-deprecated-declarations"
        )
    endif()

    # Add include directories for mujoco_simulate and joystick headers
    target_include_directories(rl_sim_mujoco PRIVATE
        library/thirdparty/mujoco_simulate
        library/thirdparty/joystick
    )

    target_link_libraries(rl_sim_mujoco
        rl_sdk observation_buffer ${YAML_CPP_LIBRARIES}
        Threads::Threads mujoco_prebuilt glfw
    )
    if(NOT USE_CMAKE)
        if(FALSE)
            target_link_libraries(rl_sim_mujoco ${catkin_LIBRARIES})
        elseif($ENV{ROS_DISTRO} MATCHES "foxy")
            ament_target_dependencies(rl_sim_mujoco
                rclcpp
                std_msgs
            )
        endif()
    endif()

    # macOS specific frameworks and libraries
    if(SYSTEM_TYPE STREQUAL "Darwin")
        # OpenMP support
        if(OPENMP_LIBRARIES)
            target_link_libraries(rl_sim_mujoco ${OPENMP_LIBRARIES})
        endif()

        # Required frameworks for GlfwCoreVideo
        target_link_libraries(rl_sim_mujoco
            "-framework CoreVideo"
            "-framework OpenGL"
            "-framework AppKit"
        )
    endif()

    install(TARGETS rl_sim_mujoco DESTINATION bin)
    message(STATUS "MuJoCo configured: ${MUJOCO_DIR}")
endif()

# ========================
# only for test
# ========================
# Unit tests are enabled in standalone CMake mode to keep Jetson deploy path lean.
if(USE_CMAKE AND BUILD_TESTING)
    enable_testing()

    add_executable(test_loop_timing_precision test/test_loop_timing_precision.cpp)
    target_link_libraries(test_loop_timing_precision Threads::Threads)
    add_test(NAME test_loop_timing_precision COMMAND test_loop_timing_precision)

    add_executable(test_loop_exception_handler test/test_loop_exception_handler.cpp)
    target_link_libraries(test_loop_exception_handler Threads::Threads)
    add_test(NAME test_loop_exception_handler COMMAND test_loop_exception_handler)

    add_executable(test_joint_mapping_validation test/test_joint_mapping_validation.cpp)
    add_test(NAME test_joint_mapping_validation COMMAND test_joint_mapping_validation)

    add_executable(test_go2_x5_control_logic test/test_go2_x5_control_logic.cpp)
    add_test(NAME test_go2_x5_control_logic COMMAND test_go2_x5_control_logic)

    add_executable(test_go2_x5_ipc_protocol test/test_go2_x5_ipc_protocol.cpp)
    add_test(NAME test_go2_x5_ipc_protocol COMMAND test_go2_x5_ipc_protocol)

    add_executable(test_go2_x5_protocol_serialization
        test/test_go2_x5_protocol_serialization.cpp
        src/rl_sar/protocol/go2_x5_protocol.cpp)
    add_test(NAME test_go2_x5_protocol_serialization COMMAND test_go2_x5_protocol_serialization)
    add_test(
        NAME test_go2_x5_arx_x5_bridge_py_compile
        COMMAND ${Python3_EXECUTABLE} -m py_compile ${CMAKE_CURRENT_SOURCE_DIR}/scripts/arx_x5_bridge.py
    )

    add_executable(test_go2_x5_arm_bridge_defaults test/test_go2_x5_arm_bridge_defaults.cpp)
    target_compile_definitions(test_go2_x5_arm_bridge_defaults PRIVATE RL_SAR_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
    add_test(NAME test_go2_x5_arm_bridge_defaults COMMAND test_go2_x5_arm_bridge_defaults)

    add_executable(test_go2_x5_config_split test/test_go2_x5_config_split.cpp)
    target_compile_definitions(test_go2_x5_config_split PRIVATE RL_SAR_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
    add_test(NAME test_go2_x5_config_split COMMAND test_go2_x5_config_split)

    add_executable(test_go2_x5_launch_defaults
        test/test_go2_x5_launch_defaults.cpp
        library/core/config/deploy_manifest_loader.cpp)
    target_compile_definitions(test_go2_x5_launch_defaults PRIVATE RL_SAR_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
    target_link_libraries(test_go2_x5_launch_defaults ${YAML_CPP_LIBRARIES} Threads::Threads)
    add_test(NAME test_go2_x5_launch_defaults COMMAND test_go2_x5_launch_defaults)

    add_executable(test_go2_x5_arm_lock_defaults test/test_go2_x5_arm_lock_defaults.cpp)
    target_compile_definitions(test_go2_x5_arm_lock_defaults PRIVATE RL_SAR_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
    add_test(NAME test_go2_x5_arm_lock_defaults COMMAND test_go2_x5_arm_lock_defaults)

    add_executable(test_go2_x5_arm_safety_guards test/test_go2_x5_arm_safety_guards.cpp)
    target_compile_definitions(test_go2_x5_arm_safety_guards PRIVATE RL_SAR_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
    add_test(NAME test_go2_x5_arm_safety_guards COMMAND test_go2_x5_arm_safety_guards)

    add_executable(test_go2_x5_safety_guard test/test_go2_x5_safety_guard.cpp src/rl_sar/go2x5/safety/go2_x5_safety_guard.cpp)
    add_test(NAME test_go2_x5_safety_guard COMMAND test_go2_x5_safety_guard)

    add_executable(test_go2_x5_arm_runtime test/test_go2_x5_arm_runtime.cpp src/rl_sar/go2x5/arm/go2_x5_arm_runtime.cpp)
    add_test(NAME test_go2_x5_arm_runtime COMMAND test_go2_x5_arm_runtime)

    add_executable(test_go2_x5_arm_bridge_runtime test/test_go2_x5_arm_bridge_runtime.cpp src/rl_sar/go2x5/arm/go2_x5_arm_bridge_runtime.cpp src/rl_sar/go2x5/safety/go2_x5_safety_guard.cpp)
    add_test(NAME test_go2_x5_arm_bridge_runtime COMMAND test_go2_x5_arm_bridge_runtime)

    add_executable(test_go2_x5_arm_output_guard test/test_go2_x5_arm_output_guard.cpp src/rl_sar/go2x5/arm/go2_x5_arm_output_guard.cpp)
    add_test(NAME test_go2_x5_arm_output_guard COMMAND test_go2_x5_arm_output_guard)

    # Config layer tests
    add_executable(test_config_loader test/test_config_loader.cpp library/core/config/config_loader.cpp)
    target_link_libraries(test_config_loader ${YAML_CPP_LIBRARIES} Threads::Threads)
    add_test(NAME test_config_loader COMMAND test_config_loader)

    add_executable(test_go2_x5_deploy_manifest_loader
        test/test_go2_x5_deploy_manifest_loader.cpp
        library/core/config/deploy_manifest_loader.cpp)
    target_link_libraries(test_go2_x5_deploy_manifest_loader ${YAML_CPP_LIBRARIES} Threads::Threads)
    add_test(NAME test_go2_x5_deploy_manifest_loader COMMAND test_go2_x5_deploy_manifest_loader)

    add_executable(test_go2_x5_protocol_contract
        test/test_go2_x5_protocol_contract.cpp
        src/rl_sar/protocol/go2_x5_protocol.cpp)
    add_test(NAME test_go2_x5_protocol_contract COMMAND test_go2_x5_protocol_contract)

    add_executable(test_go2_x5_manifest_contract
        test/test_go2_x5_manifest_contract.cpp
        library/core/config/deploy_manifest_loader.cpp
        library/core/config/deploy_manifest_runtime.cpp)
    target_link_libraries(test_go2_x5_manifest_contract ${YAML_CPP_LIBRARIES} Threads::Threads)
    add_test(NAME test_go2_x5_manifest_contract COMMAND test_go2_x5_manifest_contract)

    add_executable(test_go2_x5_supervisor_contract
        test/test_go2_x5_supervisor_contract.cpp
        src/rl_sar/runtime/supervisor/go2_x5_supervisor.cpp)
    add_test(NAME test_go2_x5_supervisor_contract COMMAND test_go2_x5_supervisor_contract)

    add_executable(test_go2_x5_coordinator_contract
        test/test_go2_x5_coordinator_contract.cpp
        src/rl_sar/runtime/coordinator/go2_x5_coordinator.cpp
        src/rl_sar/protocol/go2_x5_protocol.cpp)
    add_test(NAME test_go2_x5_coordinator_contract COMMAND $<TARGET_FILE:test_go2_x5_coordinator_contract>)

    add_executable(test_go2_x5_freshness_contract
        test/test_go2_x5_freshness_contract.cpp
        src/rl_sar/protocol/go2_x5_protocol.cpp
        src/rl_sar/runtime/supervisor/go2_x5_supervisor.cpp)
    add_test(NAME test_go2_x5_freshness_contract COMMAND test_go2_x5_freshness_contract)

    add_executable(test_go2_x5_runtime_smoke
        test/test_go2_x5_runtime_smoke.cpp
        library/core/config/deploy_manifest_loader.cpp
        library/core/config/deploy_manifest_runtime.cpp
        src/rl_sar/protocol/go2_x5_protocol.cpp
        src/rl_sar/runtime/supervisor/go2_x5_supervisor.cpp)
    target_compile_definitions(test_go2_x5_runtime_smoke PRIVATE RL_SAR_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
    target_link_libraries(test_go2_x5_runtime_smoke ${YAML_CPP_LIBRARIES} Threads::Threads)
    add_test(NAME test_go2_x5_runtime_smoke COMMAND test_go2_x5_runtime_smoke)

    # Arm transport tests
    add_executable(test_arm_transport test/test_arm_transport.cpp src/rl_sar/go2x5/arm/go2_x5_arm_transport.cpp src/rl_sar/go2x5/safety/go2_x5_safety_guard.cpp)
    target_link_libraries(test_arm_transport ${YAML_CPP_LIBRARIES} Threads::Threads)
    add_test(NAME test_arm_transport COMMAND test_arm_transport)

    # State manager tests
    add_executable(test_state_manager test/test_state_manager.cpp src/rl_sar/go2x5/state/go2_x5_state_manager.cpp)
    target_link_libraries(test_state_manager Threads::Threads)
    add_test(NAME test_state_manager COMMAND test_state_manager)

    # Adapter tests
    if(UNITREE_SDK2_AVAILABLE)
        add_executable(test_unitree_adapter_contract
            test/test_unitree_adapter_contract.cpp
            src/rl_sar/adapters/unitree/unitree_adapter.cpp
            src/rl_sar/protocol/go2_x5_protocol.cpp)
        target_include_directories(test_unitree_adapter_contract BEFORE PRIVATE
            ${UNITREE_SDK2_THIRDPARTY_INCLUDE}
            ${UNITREE_SDK2_THIRDPARTY_DDSCXX_INCLUDE}
        )
        target_link_libraries(test_unitree_adapter_contract
            unitree_sdk2
            Threads::Threads
        )
        target_compile_definitions(test_unitree_adapter_contract PRIVATE UNITREE_SDK2_AVAILABLE)
        add_test(NAME test_unitree_adapter_contract COMMAND test_unitree_adapter_contract)
    endif()

    # ARX adapter tests
    add_executable(test_arm_adapter_contract
        test/test_arm_adapter_contract.cpp
        src/rl_sar/adapters/arx/arx_adapter.cpp
        src/rl_sar/protocol/go2_x5_protocol.cpp)
    target_include_directories(test_arm_adapter_contract PRIVATE include)
    target_link_libraries(test_arm_adapter_contract
        gtest
        gtest_main
        Threads::Threads
        ${CMAKE_DL_LIBS}
    )
    add_test(NAME test_arm_adapter_contract COMMAND test_arm_adapter_contract)

    # Diagnostics tests
    add_executable(test_drift_recorder
        test/test_drift_recorder.cpp
        src/rl_sar/diagnostics/drift_recorder.cpp)
    target_include_directories(test_drift_recorder PRIVATE include)
    target_link_libraries(test_drift_recorder Threads::Threads)
    add_test(NAME test_drift_recorder COMMAND test_drift_recorder)

    add_executable(test_diagnostics_publisher
        test/test_diagnostics_publisher.cpp
        src/rl_sar/diagnostics/diagnostics_publisher.cpp)
    target_include_directories(test_diagnostics_publisher PRIVATE include)
    target_link_libraries(test_diagnostics_publisher Threads::Threads)
    add_test(NAME test_diagnostics_publisher COMMAND test_diagnostics_publisher)

    # Event logger tests
    add_executable(test_event_logger
        test/test_event_logger.cpp
        src/rl_sar/logger/event_logger.cpp)
    target_include_directories(test_event_logger PRIVATE include)
    target_link_libraries(test_event_logger Threads::Threads)
    add_test(NAME test_event_logger COMMAND test_event_logger)
endif()

# add_executable(test_observation_buffer test/test_observation_buffer.cpp)
# target_link_libraries(test_observation_buffer
#     observation_buffer
# )
# install(TARGETS test_observation_buffer DESTINATION lib/${PROJECT_NAME})
# add_executable(test_vector_math test/test_vector_math.cpp)
# install(TARGETS test_vector_math DESTINATION lib/${PROJECT_NAME})

# add_executable(test_inference_stability test/test_inference_stability.cpp)
# target_link_libraries(test_inference_stability
#     inference_runtime
#     ${TORCH_LIBRARIES}
#     ${ONNX_RUNTIME_LIB}
# )
# install(TARGETS test_inference_stability DESTINATION lib/${PROJECT_NAME})

message(STATUS "========================================")
message(STATUS "       RL_SAR Build Configuration       ")
message(STATUS "========================================")
message(STATUS "Project: ${PROJECT_NAME} v${PROJECT_VERSION}")
message(STATUS "System: ${SYSTEM_TYPE}")
message(STATUS "Architecture: ${CMAKE_SYSTEM_PROCESSOR}")
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "Compiler: ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")
message(STATUS "USE_CMAKE: ${USE_CMAKE}")
message(STATUS "----------------------------------------")
message(STATUS "Inference Backend Configuration:")
message(STATUS "USE_ONNX (required): ${USE_ONNX}")
message(STATUS "ENABLE_TORCH_BACKEND (optional): ${ENABLE_TORCH_BACKEND}")
message(STATUS "USE_TORCH: ${USE_TORCH}")
if(USE_ONNX)
    if(ONNX_RUNTIME_VERSION)
        message(STATUS "ONNX Runtime version: ${ONNX_RUNTIME_VERSION}")
    elseif(onnxruntime_VERSION)
        message(STATUS "ONNX Runtime version: ${onnxruntime_VERSION}")
    else()
        message(STATUS "ONNX Runtime: Available")
    endif()
    if(IS_JETSON)
        message(STATUS "Platform: Jetson (ARM64) - ONNX Runtime recommended")
    endif()
endif()
if(USE_TORCH)
    if(Torch_VERSION)
        message(STATUS "LibTorch version: ${Torch_VERSION}")
    else()
        message(STATUS "LibTorch: Available (optional backend)")
    endif()
endif()
message(STATUS "----------------------------------------")
message(STATUS "Simulator Configuration:")
message(STATUS "USE_MUJOCO: ${USE_MUJOCO}")
if(USE_MUJOCO)
    message(STATUS "MuJoCo: Enabled")
endif()
message(STATUS "========================================")
