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
        find_package(rclcpp REQUIRED)
        find_package(std_msgs REQUIRED)
        find_package(rclpy REQUIRED)
        find_package(geometry_msgs REQUIRED)
        ament_package()
    else()
        message(FATAL_ERROR "rl_ras_n supports ROS2 Foxy only. Current ROS_DISTRO=$ENV{ROS_DISTRO}")
    endif()
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
set(_DEFAULT_UNITREE_SDK2_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/library/thirdparty/robot_sdk/unitree/unitree_sdk2")
if(NOT EXISTS "${_DEFAULT_UNITREE_SDK2_ROOT}/CMakeLists.txt")
    set(_WORKSPACE_UNITREE_SDK2_ROOT "${PROJECT_ROOT_DIR}/../unitree_sdk2")
    if(EXISTS "${_WORKSPACE_UNITREE_SDK2_ROOT}/CMakeLists.txt")
        set(_DEFAULT_UNITREE_SDK2_ROOT "${_WORKSPACE_UNITREE_SDK2_ROOT}")
    endif()
endif()
set(UNITREE_SDK2_ROOT "${_DEFAULT_UNITREE_SDK2_ROOT}" CACHE PATH
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
    .
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
    include/rl_sar/state_estimation
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

# library for deterministic deployment observations
add_library(observation_builder src/rl_sar/observation/observation_builder.cpp)
if(NOT USE_CMAKE)
    if($ENV{ROS_DISTRO} MATCHES "foxy")
        install(TARGETS observation_builder DESTINATION lib/${PROJECT_NAME})
    endif()
endif()

# library for raw drift data capture
add_library(drift_data_recorder src/rl_sar/diagnostics/drift_data_recorder.cpp)
target_link_libraries(drift_data_recorder PUBLIC Threads::Threads)
if(NOT USE_CMAKE)
    if($ENV{ROS_DISTRO} MATCHES "foxy")
        install(TARGETS drift_data_recorder DESTINATION lib/${PROJECT_NAME})
    endif()
endif()

# library for diagnostic aggregation
add_library(diagnostic_aggregator src/rl_sar/diagnostics/diagnostic_aggregator.cpp)
target_link_libraries(diagnostic_aggregator PUBLIC Threads::Threads)
if(NOT USE_CMAKE)
    if($ENV{ROS_DISTRO} MATCHES "foxy")
        install(TARGETS diagnostic_aggregator DESTINATION lib/${PROJECT_NAME})
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
    add_library(unitree_adapter
        src/rl_sar/adapters/unitree/unitree_adapter.cpp
        src/rl_sar/state_estimation/velocity_estimator.cpp)
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
        # State Estimation
        src/rl_sar/state_estimation/velocity_estimator.cpp
        # Runtime
        src/rl_sar/protocol/go2_x5_protocol.cpp
        src/rl_sar/runtime/coordinator/go2_x5_coordinator.cpp
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
        arx_adapter
        rl_sdk
        observation_buffer
        observation_builder
        drift_data_recorder
        diagnostic_aggregator
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

# External ARX bridge script removed. Arm control is provided by ArxAdapter only.

if(NOT USE_CMAKE)
    if($ENV{ROS_DISTRO} MATCHES "foxy")
        install(
            DIRECTORY launch
            DESTINATION share/${PROJECT_NAME}
        )
        install(
            DIRECTORY ${PROJECT_ROOT_DIR}/deploy ${PROJECT_ROOT_DIR}/policy
            DESTINATION share/${PROJECT_NAME}
        )
    endif()
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

    # Bridge-related tests removed - InProcessSdk only

    add_executable(test_go2_x5_config_split test/test_go2_x5_config_split.cpp)
    target_compile_definitions(test_go2_x5_config_split PRIVATE RL_SAR_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
    add_test(NAME test_go2_x5_config_split COMMAND test_go2_x5_config_split)

    add_executable(test_go2_x5_launch_defaults
        test/test_go2_x5_launch_defaults.cpp
        library/core/config/deploy_manifest_loader.cpp)
    target_compile_definitions(test_go2_x5_launch_defaults PRIVATE RL_SAR_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
    target_link_libraries(test_go2_x5_launch_defaults ${YAML_CPP_LIBRARIES} Threads::Threads)
    add_test(NAME test_go2_x5_launch_defaults COMMAND test_go2_x5_launch_defaults)

    add_executable(test_go2_x5_keyboard_input_contract test/test_go2_x5_keyboard_input_contract.cpp)
    target_compile_definitions(test_go2_x5_keyboard_input_contract PRIVATE RL_SAR_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
    add_test(NAME test_go2_x5_keyboard_input_contract COMMAND test_go2_x5_keyboard_input_contract)

    add_executable(test_go2_x5_startup_safety_contract test/test_go2_x5_startup_safety_contract.cpp)
    target_compile_definitions(test_go2_x5_startup_safety_contract PRIVATE RL_SAR_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
    add_test(NAME test_go2_x5_startup_safety_contract COMMAND test_go2_x5_startup_safety_contract)

    add_executable(test_go2_x5_arm_lock_defaults
        test/test_go2_x5_arm_lock_defaults.cpp
        src/rl_sar/go2x5/arm/go2_x5_arm_runtime.cpp)
    add_test(NAME test_go2_x5_arm_lock_defaults COMMAND test_go2_x5_arm_lock_defaults)

    add_executable(test_go2_x5_arm_safety_guards
        test/test_go2_x5_arm_safety_guards.cpp
        src/rl_sar/go2x5/arm/go2_x5_arm_output_guard.cpp)
    add_test(NAME test_go2_x5_arm_safety_guards COMMAND test_go2_x5_arm_safety_guards)

    add_executable(test_go2_x5_safety_guard test/test_go2_x5_safety_guard.cpp src/rl_sar/go2x5/safety/go2_x5_safety_guard.cpp)
    add_test(NAME test_go2_x5_safety_guard COMMAND test_go2_x5_safety_guard)

    add_executable(test_go2_x5_arm_runtime test/test_go2_x5_arm_runtime.cpp src/rl_sar/go2x5/arm/go2_x5_arm_runtime.cpp)
    add_test(NAME test_go2_x5_arm_runtime COMMAND test_go2_x5_arm_runtime)

    # Bridge runtime test removed - InProcessSdk only

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

    add_executable(test_go2_x5_runtime_validator
        test/test_go2_x5_runtime_validator.cpp)
    target_link_libraries(test_go2_x5_runtime_validator Threads::Threads)
    add_test(NAME test_go2_x5_runtime_validator COMMAND test_go2_x5_runtime_validator)

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
            src/rl_sar/state_estimation/velocity_estimator.cpp
            src/rl_sar/protocol/go2_x5_protocol.cpp)
        target_include_directories(test_unitree_adapter_contract BEFORE PRIVATE
            ${UNITREE_SDK2_THIRDPARTY_INCLUDE}
            ${UNITREE_SDK2_THIRDPARTY_DDSCXX_INCLUDE}
        )
        target_link_libraries(test_unitree_adapter_contract
            gtest
            gtest_main
            unitree_sdk2
            Threads::Threads
        )
        set_target_properties(test_unitree_adapter_contract PROPERTIES
            BUILD_RPATH "${CMAKE_INSTALL_RPATH};${UNITREE_SDK2_ROOT}/thirdparty/lib/${ARCH_DIR};${UNITREE_SDK2_ROOT}/lib/${ARCH_DIR}"
            INSTALL_RPATH "${CMAKE_INSTALL_RPATH};${UNITREE_SDK2_ROOT}/thirdparty/lib/${ARCH_DIR};${UNITREE_SDK2_ROOT}/lib/${ARCH_DIR}"
        )
        target_compile_definitions(test_unitree_adapter_contract PRIVATE UNITREE_SDK2_AVAILABLE)
        add_test(NAME test_unitree_adapter_contract COMMAND test_unitree_adapter_contract)
    endif()

    # ARX adapter tests
    add_library(fake_arx_hardware SHARED test/fake_arx_hardware.cpp)
    add_library(fake_arx_hardware_no_ec_queries SHARED test/fake_arx_hardware.cpp)
    target_compile_definitions(fake_arx_hardware_no_ec_queries PRIVATE RL_SAR_TEST_OMIT_EC_QUERY_SYMBOLS=1)
    add_executable(test_arm_adapter_contract
        test/test_arm_adapter_contract.cpp
        src/rl_sar/adapters/arx/arx_adapter.cpp
        src/rl_sar/protocol/go2_x5_protocol.cpp)
    target_include_directories(test_arm_adapter_contract PRIVATE include)
    add_dependencies(test_arm_adapter_contract fake_arx_hardware fake_arx_hardware_no_ec_queries)
    target_compile_definitions(test_arm_adapter_contract PRIVATE
        RL_SAR_TEST_FAKE_ARX_HARDWARE_LIB=\"$<TARGET_FILE:fake_arx_hardware>\"
        RL_SAR_TEST_FAKE_ARX_HARDWARE_LIB_NO_EC_QUERY=\"$<TARGET_FILE:fake_arx_hardware_no_ec_queries>\")
    target_link_libraries(test_arm_adapter_contract
        gtest
        gtest_main
        Threads::Threads
        ${CMAKE_DL_LIBS}
    )
    add_test(NAME test_arm_adapter_contract COMMAND test_arm_adapter_contract)

    add_executable(test_observation_builder test/test_observation_builder.cpp)
    target_include_directories(test_observation_builder PRIVATE include)
    target_link_libraries(test_observation_builder
        gtest
        gtest_main
        observation_builder
        rl_sdk
        observation_buffer
        Threads::Threads
        ${YAML_CPP_LIBRARIES}
    )
    add_test(NAME test_observation_builder COMMAND test_observation_builder)

    add_executable(test_velocity_estimator
        test/test_velocity_estimator.cpp
        src/rl_sar/state_estimation/velocity_estimator.cpp)
    target_include_directories(test_velocity_estimator PRIVATE include)
    target_link_libraries(test_velocity_estimator
        gtest
        gtest_main
        Threads::Threads
    )
    add_test(NAME test_velocity_estimator COMMAND test_velocity_estimator)

    # Diagnostics tests
    add_executable(test_drift_data_recorder
        test/test_drift_data_recorder.cpp
        src/rl_sar/diagnostics/drift_data_recorder.cpp)
    target_include_directories(test_drift_data_recorder PRIVATE include)
    target_link_libraries(test_drift_data_recorder Threads::Threads)
    add_test(NAME test_drift_data_recorder COMMAND test_drift_data_recorder)

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

    add_executable(test_diagnostic_aggregator
        test/test_diagnostic_aggregator.cpp
        src/rl_sar/diagnostics/diagnostic_aggregator.cpp)
    target_include_directories(test_diagnostic_aggregator PRIVATE include)
    target_link_libraries(test_diagnostic_aggregator Threads::Threads)
    add_test(NAME test_diagnostic_aggregator COMMAND test_diagnostic_aggregator)

    # Event logger tests
    add_executable(test_event_logger
        test/test_event_logger.cpp
        src/rl_sar/logger/event_logger.cpp)
    target_include_directories(test_event_logger PRIVATE include)
    target_link_libraries(test_event_logger Threads::Threads)
    add_test(NAME test_event_logger COMMAND test_event_logger)

    # ========================================================================
    # Jetson Hardware Check Tests (安全模式 - 只检测，不控制)
    # ========================================================================
    # ⚠️ 安全承诺：这些测试只检测硬件接口，不发送任何控制命令
    # 适用于Jetson安装在机器人身上的场景

    message(STATUS "Adding Jetson hardware check tests (SAFE MODE)...")

    # Hardware interface detection (read-only, no control)
    add_executable(test_hardware_interfaces
        test/hardware/test_hardware_interfaces.cpp)
    target_include_directories(test_hardware_interfaces PRIVATE include)
    target_link_libraries(test_hardware_interfaces ${YAML_CPP_LIBRARIES} ${CMAKE_DL_LIBS})

    # Observation snapshot generator (for training-deployment consistency)
    add_executable(test_observation_snapshot
        test/integration/test_observation_snapshot.cpp
        src/rl_sar/observation/observation_builder.cpp)
    target_include_directories(test_observation_snapshot PRIVATE include)
    target_link_libraries(test_observation_snapshot Threads::Threads)

    message(STATUS "Jetson hardware check tests: test_hardware_interfaces, test_observation_snapshot")
    message(STATUS "  Safe mode: Detection only, NO control commands sent")
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
message(STATUS "Mode: sim2real only")
message(STATUS "========================================")
