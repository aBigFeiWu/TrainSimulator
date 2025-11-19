#include "Simulator/TrainSimulator.h"
#include "TrainSimulatorAPI.h"
#include "TrajKit/DataTypes.h" // 需要包含此头文件以使用C++的GeodeticPoint

// extern "C" 块确保正确的链接
extern "C" {

// 使用 void* 作为句柄来隐藏 C++ 的具体实现
API_DECL void* CreateSimulator(SimulatorConfig_C config) {
    try {
        SimulatorConfiguration cpp_config;
        cpp_config.route_file = config.route_file_path;
        cpp_config.ip = config.ip;
        cpp_config.port = config.port;
        cpp_config.SIMULATION_INTERVAL_MS = config.simulation_interval_ms;

        // 设置默认的车辆参数
        cpp_config.test_vehicle = {
            config.TrainType, config.MaxSpeed, config.TrainLong, {config.Resistance[0], config.Resistance[1], config.Resistance[2]}, config.TractionAcceleration, config.BrakeAcceleration
        };
        // 设置默认的指令参数
        cpp_config.simulation_duration = config.simulation_duration;
        cpp_config.simulation_start_time = config.simulation_start_time;
        cpp_config.trajectory_ID = config.trajectory_ID;
        cpp_config.trajectory_type = config.trajectory_type;
        cpp_config.trajectory_ID_user2 = config.trajectory_ID_user2;
        cpp_config.trajectory_type_user2 = config.trajectory_type_user2;
        cpp_config.enable_second_user = (config.enable_second_user != 0);

        auto simulator = new TrainSimulator(cpp_config);

        // 根据 SimMode 设置初始控制模式
        simulator->set_control_mode(static_cast<TrainController::ControlMode>(config.SimMode));
        return simulator;
    } catch (const std::exception& e) {
        // 真实项目中应该有更好的错误处理机制
        std::cerr << "Error creating simulator: " << e.what() << std::endl;
        return nullptr;
    }
}

API_DECL void DestroySimulator(void* simulator_handle) {
    if (simulator_handle) {
        delete static_cast<TrainSimulator*>(simulator_handle);
    }
}

API_DECL bool StartSimulation(void* simulator_handle) {
    if (!simulator_handle) return false;
    auto sim = static_cast<TrainSimulator*>(simulator_handle);
    if(sim->start_simulation()) {
        sim->run_simulation_non_blocking();
        return true;
    }
    return false;
}

API_DECL bool StopSimulation(void* simulator_handle) {
    if (!simulator_handle) return false;
    return static_cast<TrainSimulator*>(simulator_handle)->stop_simulation();
}

API_DECL void SetControlMode(void* simulator_handle, int mode) {
    if (!simulator_handle) return;
    auto sim = static_cast<TrainSimulator*>(simulator_handle);
    sim->set_control_mode(static_cast<TrainController::ControlMode>(mode));
}

API_DECL void SetControlLevel(void* simulator_handle, int level) {
    if (!simulator_handle) return;
    auto sim = static_cast<TrainSimulator*>(simulator_handle);
    sim->set_control_level(static_cast<TrainController::ControlLevel>(level));
}

API_DECL KinematicState_C GetCurrentState(void* simulator_handle) {
    KinematicState_C state_c = {0};
    if (simulator_handle) {
        auto sim = static_cast<TrainSimulator*>(simulator_handle);
        const KinematicState& state_cpp = sim->getCurrentState(); // 假设 TrainSimulator 有这个方法
        state_c.position = state_cpp.position;
        state_c.velocity = state_cpp.velocity;
        state_c.acceleration = state_cpp.acceleration;
        state_c.jerk = state_cpp.jerk;
    }
    return state_c;
}


// --- 新增API函数的实现 ---

API_DECL double GetCurrentSpeed(void* simulator_handle) {
    if (simulator_handle) {
        auto sim = static_cast<TrainSimulator*>(simulator_handle);
        return sim->getCurrentSpeed();
    }
    return 0.0;
}

API_DECL GeodeticPoint_C GetCurrentPositionBLH(void* simulator_handle) {
    GeodeticPoint_C pos_c = {0.0, 0.0, 0.0};
    if (simulator_handle) {
        auto sim = static_cast<TrainSimulator*>(simulator_handle);
        // 调用我们之前在 TrainSimulator 类中添加的函数
        const GeodeticPoint pos_cpp = sim->getCurrentPositionBLH();
        pos_c.lon = pos_cpp.lon;
        pos_c.lat = pos_cpp.lat;
        pos_c.alt = pos_cpp.alt;
    }
    return pos_c;
}

} // extern "C"
