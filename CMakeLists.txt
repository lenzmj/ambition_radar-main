cmake_minimum_required(VERSION 3.12)
project(HikPikachu)

set(CMAKE_CXX_STANDARD 17)

option(AMBITION_ENABLE_TRT "Build TensorRT YOLO backend (needs CUDA + libnvinfer)" ON)

# 1. 找 OpenCV和Eigen3（kalman用）
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenVINO REQUIRED)
find_package(yaml-cpp REQUIRED)

# 2. 找海康 SDK (根据你电脑实际路径修改)
set(MVS_INCLUDE_DIR /opt/MVS/include)
set(MVS_LIBRARY_DIR /opt/MVS/lib/64)

# cam_to_ray / handeye 在 Tools/Calib 下多一层目录
file(GLOB AMB_SOURCES CONFIGURE_DEPENDS
    "src/*.cpp"
    "src/*/*.cpp"
    "src/*/*/*.cpp"
    "src/*/*/*/*.cpp"
)
# 手眼工具单独可执行文件，不链进主程序 app
list(FILTER AMB_SOURCES EXCLUDE REGEX ".*/Tools/Calib/handeye/.*")
list(FILTER AMB_SOURCES EXCLUDE REGEX ".*/Tools/Calib/ray_to_gimbal/.*")

set(AMBITION_TRT_READY FALSE)
if(AMBITION_ENABLE_TRT)
    find_package(CUDAToolkit QUIET)
    find_library(NVINFER_LIBRARY nvinfer)
    if(CUDAToolkit_FOUND AND NVINFER_LIBRARY)
        set(AMBITION_TRT_READY TRUE)
    else()
        message(WARNING "AMBITION_ENABLE_TRT is ON but CUDAToolkit or libnvinfer not found; building without TensorRT backend.")
    endif()
endif()

if(NOT AMBITION_TRT_READY)
    list(FILTER AMB_SOURCES EXCLUDE REGEX ".*/yolo_tensorrt\\.cpp$")
endif()

# 3. 头文件
include_directories(
    ./include
    ${OpenCV_INCLUDE_DIRS}
    ${MVS_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIRS}
    ${OpenVINO_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIR}
)

# 4. 库文件
link_directories(${MVS_LIBRARY_DIR})

# 5. 编译源
add_executable(app ${AMB_SOURCES})

# 6. 链接库
target_link_libraries(app PRIVATE
    ${OpenCV_LIBS}
    openvino::runtime
    yaml-cpp
    MvCameraControl
    pthread
    dl
)

if(AMBITION_TRT_READY)
    target_compile_definitions(app PRIVATE AMBITION_WITH_TENSORRT=1)
    target_include_directories(app PRIVATE ${CUDAToolkit_INCLUDE_DIRS})
    target_link_libraries(app PRIVATE ${NVINFER_LIBRARY} CUDA::cudart)
    message(STATUS "TensorRT backend enabled: ${NVINFER_LIBRARY}")
endif()

# ---------------------------------------------------------------------------
# 手眼标定：src/Tools/Calib/handeye（独立可执行文件）
# ---------------------------------------------------------------------------
add_executable(handeye_capture
    src/Tools/Calib/handeye/handeye_capture.cpp
    src/Tools/Hik/HikDriver.cpp
    src/Serial/SerialDriver.cpp
    src/Serial/crc.cpp
)
target_link_libraries(handeye_capture PRIVATE
    ${OpenCV_LIBS}
    yaml-cpp
    MvCameraControl
    pthread
    dl
)

add_executable(handeye_calibrate
    src/Tools/Calib/handeye/handeye_calibrate.cpp
)
target_link_libraries(handeye_calibrate PRIVATE
    ${OpenCV_LIBS}
    yaml-cpp
)

# ---------------------------------------------------------------------------
# 多距离 rpy → 改良 ray_to_gimbal：src/Tools/Calib/ray_to_gimbal
# ---------------------------------------------------------------------------
add_executable(ray_to_gimbal_calibrate
    src/Tools/Calib/ray_to_gimbal/ray_to_gimbal_calibrate.cpp
)
target_link_libraries(ray_to_gimbal_calibrate PRIVATE
    yaml-cpp
)
