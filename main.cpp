#include "Simulator/TrainSimulator.h"
#include "Simulator/SimulatorConfiguration.h"
#include "DynamicModel/TestVehicle.h"
#include <chrono>
#include <thread>
#include <iostream>

namespace {
// 计算自 2006-01-01 00:00:00 起的毫秒数
long long milliseconds_since_2006() {
    using clock = std::chrono::system_clock;
    const auto now = clock::now();
    std::tm base_tm{};
    base_tm.tm_year = 2006 - 1900;
    base_tm.tm_mon = 0;
    base_tm.tm_mday = 1;
    const auto base_time = clock::from_time_t(std::mktime(&base_tm));
    return std::chrono::duration_cast<std::chrono::milliseconds>(now - base_time).count();
}
} // namespace

int main() {
    try {
        // 车辆参数示例
        TrainInfo vehicle{};
        vehicle.trainType = L"DemoTrain";
        vehicle.maxSpeed = 120.0;            // km/h
        vehicle.trainLong = 200.0;           // m
        vehicle.resistanceCoefficients[0] = 2.28;
        vehicle.resistanceCoefficients[1] = 0.0293;
        vehicle.resistanceCoefficients[2] = 0.000178;
        vehicle.tractionAcceleration = 0.6;  // m/s^2
        vehicle.brakingAcceleration = -0.9;  // m/s^2

        SimulatorConfiguration config{};
        config.test_vehicle = vehicle;
        config.route_file = L"D:\\HOME\\TrainSimulator_Dll\\TrainSimulator\\cmake-build-debug\\trajectory_BLH.txt";
        config.ip = L"10.129.1.100";
        config.port = 9988;
        config.SIMULATION_INTERVAL_MS = 20;

        config.simulation_start_time = milliseconds_since_2006();
        config.simulation_duration = 600'000; // 60s
        config.trajectory_ID = 1;            // 车头
        config.trajectory_type = 1;
        config.trajectory_ID_user2 = 2;      // 车尾
        config.trajectory_type_user2 = 1;
        config.enable_second_user = true;    // 双用户（车头+车尾）

        TrainSimulator simulator(config);

        if (!simulator.start_simulation()) {
            std::cerr << "Start command failed, exiting." << std::endl;
            return 1;
        }



        simulator.run_simulation_non_blocking();


        std::this_thread::sleep_for(std::chrono::seconds(600));
        simulator.stop_simulation();
    } catch (const std::exception& ex) {
        std::cerr << "Exception: " << ex.what() << std::endl;
        return 1;
    }

    return 0;
}
