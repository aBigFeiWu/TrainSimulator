cmake_minimum_required(VERSION 3.21)
project(TrainSimulator CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add M_PI and related macros on Windows
add_definitions(-D_USE_MATH_DEFINES)

# Gather all source files
file(GLOB_RECURSE ALL_SOURCES
        "DynamicModel/*.cpp"
        "TrajKit/*.cpp"
        "TrainCommunicator/*.cpp"
        "Simulator/*.cpp"
)

if(MSVC)
    add_compile_options(/utf-8)
endif()
if (MINGW)
    # Ensure UTF-8 literals and runtime output on MinGW
    add_compile_options(-finput-charset=UTF-8 -fexec-charset=UTF-8)
endif()

add_library(TrainSimulator SHARED
        ${ALL_SOURCES}
        TrainSimulatorAPI.cpp
)

target_include_directories(TrainSimulator PUBLIC ${CMAKE_SOURCE_DIR})

# Export macro when building the shared library
target_compile_definitions(TrainSimulator PRIVATE TRAINSIMULATOR_EXPORTS)

# Avoid Windows header macros clashing with std::min/std::max
target_compile_definitions(TrainSimulator PRIVATE NOMINMAX)

# Windows: export all symbols so downstream consumers can link without explicit dllexport, and link sockets lib
if (WIN32)
    set_target_properties(TrainSimulator PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS ON)
    target_link_libraries(TrainSimulator PRIVATE ws2_32)
endif()

# Copy resource files into build dir for runtime access
file(COPY Resources/ DESTINATION ${CMAKE_BINARY_DIR})
message(STATUS "Resource files will be copied to ${CMAKE_BINARY_DIR}")

# Local test executable
add_executable(TrainSimulatorApp
        main.cpp
)
target_link_libraries(TrainSimulatorApp PRIVATE TrainSimulator)
target_compile_definitions(TrainSimulatorApp PRIVATE NOMINMAX)
target_include_directories(TrainSimulatorApp PRIVATE ${CMAKE_SOURCE_DIR})
