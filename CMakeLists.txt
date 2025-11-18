cmake_minimum_required(VERSION 3.21)
project(TrainSimulator CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 为 M_PI 等宏添加定义
add_definitions(-D_USE_MATH_DEFINES)

# --- 自动查找所有源文件 (.cpp) ---
# 这会自动包含 main.cpp, 以及 DynamicModel, TrajKit, TrainCommunicator
# 目录下的所有 .cpp 文件。
file(GLOB_RECURSE ALL_SOURCES
        "DynamicModel/*.cpp"
        "TrajKit/*.cpp"
        "TrainCommunicator/*.cpp"
        "Simulator/*.cpp"
)

if(MSVC)
    add_compile_options(/utf-8)
endif()

add_library(TrainSimulator SHARED
        ${ALL_SOURCES}
        TrainSimulatorAPI.cpp
)

target_include_directories(TrainSimulator PUBLIC ${CMAKE_SOURCE_DIR})

# 当我们构建这个库时，定义 TRAINSIMULATOR_EXPORTS
target_compile_definitions(TrainSimulator PRIVATE TRAINSIMULATOR_EXPORTS)

# 在你的 add_executable 或 add_library 之后
target_compile_definitions(TrainSimulator PRIVATE NOMINMAX)

# --- 自动处理资源文件 ---
# 这条命令会在每次编译后，将 Resources 目录下的所有文件
# 复制到生成的可执行文件所在的目录 (例如 cmake-build-debug/)
# 这样，程序在运行时就能用相对路径 "trajectory.txt" 找到文件。
file(COPY Resources/ DESTINATION ${CMAKE_BINARY_DIR})
message(STATUS "Resource files will be copied to ${CMAKE_BINARY_DIR}")

# 如果未来需要链接外部库，可以在这里添加
# target_link_libraries(TrainSimulator PRIVATE some_library)