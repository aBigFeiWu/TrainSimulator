// File: Simulator\TrainSimulator.h (修改)

#ifndef TRAINSIMULATOR_H
#define TRAINSIMULATOR_H

#include "DynamicModel/TrainController.h"
#include "TrajKit/Route.h"
#include "TrainCommunicator/UdpCommunicator.h"
#include "TrainCommunicator/MillisecondTimer.h"
#include "TrainCommunicator/Protocol.h"
#include "SimulatorConfiguration.h"
#include "TrajKit/GeoUtils.h"

#include <array>
#include <string>
#include <memory>

class TrainSimulator {
public:
    explicit TrainSimulator(const SimulatorConfiguration& config);
    ~TrainSimulator();

    // --- 仿真生命周期控制 ---
    bool start_simulation();
    void run_simulation_non_blocking(); // 非阻塞运行
    bool stop_simulation();

    // --- 状态与模式控制 ---
    void print_current_status() const;
    void set_control_mode(TrainController::ControlMode mode);
    void set_control_level(TrainController::ControlLevel level);

    const KinematicState& getCurrentState() const;

    double getCurrentSpeed() const;
    // 新增：获取当前BLH坐标的函数
    GeodeticPoint getCurrentPositionBLH() const;
    double getSimulationTime () const;

private:
    long long simulation_start_time_;

    UdpCommunicator udp_comm;
    MillisecondTimer timer;
    Route route;
    std::unique_ptr<TrainController> train_controller_ptr;
    SimulatorConfiguration config_;
    std::array<unsigned long long, 2> trajectory_sequence_numbers_;
};

#endif // TRAINSIMULATOR_H
