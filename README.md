# TrainSimulator
用于车辆运动规划和轨迹生成的模拟器。

## 功能
- C++20 列车动力学、规划逻辑，以及UDP轨迹广播功能
- 基于CMake的共享库构建（简单说：用CMake工具编译代码成可调用的库文件）
- 运行时会加载“路线和轨迹资源文件”（放在Resources文件夹里）

## 构建方式（编译代码）
```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build