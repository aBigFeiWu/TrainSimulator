#include "TrainSimulator.h"
#include <iostream>
#include <functional>
#include <chrono>
#include <thread>
#include <stdexcept>

TrainSimulator::TrainSimulator(const SimulatorConfiguration& config)
    : config_(config), // Existing
      udp_comm(config.ip, config.port), // Existing
      trajectory_sequence_numbers_{0, 0}, // Existing (now per-user)
      simulation_start_time_(config.simulation_start_time)
{
    // 为第二用户应用安全默认值（如果调用方未赋值）
    if (config_.trajectory_ID_user2 == 0) {
        config_.trajectory_ID_user2 = config_.trajectory_ID + 1;
    }
    if (config_.trajectory_type_user2 == 0) {
        config_.trajectory_type_user2 = config_.trajectory_type;
    }

    std::cout << "正在使用配置构造TrainSimulator..." << std::endl;
    if (!route.loadFromFile(config_.route_file)) {
        throw std::runtime_error("加载路线文件失败：" + config_.route_file);
    }
    std::cout << "路线加载成功。总距离：" << route.getTotalDistance() << "米。" << std::endl;

    train_controller_ptr = std::make_unique<TrainController>(config_.test_vehicle, route.getTotalDistance());
    std::cout << "列车控制器初始化完成。" << std::endl;

    timer.setInterval(config_.SIMULATION_INTERVAL_MS);
    timer.setCallback([this]() {
        const double dt = static_cast<double>(config_.SIMULATION_INTERVAL_MS) / 1000.0;
        // 更新每个用户的轨迹序列号
        const unsigned long long seq_user1 = ++trajectory_sequence_numbers_[0];
        const unsigned long long seq_user2 = ++trajectory_sequence_numbers_[1];

        train_controller_ptr->update(dt);
        const KinematicState& state_1d = train_controller_ptr->getCurrentState();

        const ECEFPoint pos_3d = route.getPositionAt(state_1d.position);
        const ECEFPoint vel_3d = route.getVelocityAt(state_1d.position, state_1d.velocity);
        const ECEFPoint acc_3d = route.getAccelerationAt(state_1d.position, state_1d.velocity, state_1d.acceleration);
        const ECEFPoint jerk_3d = route.getJerkAt(state_1d.position, state_1d.velocity, state_1d.acceleration, state_1d.jerk);

        DualTrajectoryData dual_packet = {};

        auto fill_user_data = [&](TrajectoryData& user_data,
                                  unsigned int trajectory_id,
                                  unsigned int trajectory_type,
                                  unsigned long long seq_number) {
            user_data.trajectory_data_seq_num = seq_number;
            user_data.trajectory_time = (seq_number - 1) * dt;
            user_data.trajectory_id = trajectory_id;
            user_data.trajectory_type = trajectory_type;

            user_data.user_pos_x = pos_3d.x; user_data.user_pos_y = pos_3d.y; user_data.user_pos_z = pos_3d.z;
            user_data.user_vel_x = vel_3d.x; user_data.user_vel_y = vel_3d.y; user_data.user_vel_z = vel_3d.z;
            user_data.user_acc_x = acc_3d.x; user_data.user_acc_y = acc_3d.y; user_data.user_acc_z = acc_3d.z;
            user_data.user_jerk_x = jerk_3d.x; user_data.user_jerk_y = jerk_3d.y; user_data.user_jerk_z = jerk_3d.z;
        };

        fill_user_data(dual_packet.user1,
                       static_cast<unsigned int>(config_.trajectory_ID),
                       static_cast<unsigned int>(config_.trajectory_type),
                       seq_user1);
        fill_user_data(dual_packet.user2,
                       static_cast<unsigned int>(config_.trajectory_ID_user2),
                       static_cast<unsigned int>(config_.trajectory_type_user2),
                       seq_user2);

        udp_comm.send(&dual_packet, sizeof(dual_packet));
    });
    std::cout << "TrainSimulator构造成功，回调已设置。" << std::endl;
}

TrainSimulator::~TrainSimulator() {
    timer.stop();
}

bool TrainSimulator::start_simulation() {
    try {
        if (!udp_comm.is_initialized()) {
            std::cerr << "错误：UDP通信器未初始化，无法发送START命令" << std::endl;
            return false;
        }
        StartCommand start_cmd = {};
        start_cmd.command_word = 0x000000000ABC0001;
        start_cmd.simulation_duration = config_.simulation_duration;
        start_cmd.simulation_start_time = config_.simulation_start_time;
        start_cmd.data_length = sizeof(StartCommand);

        const ECEFPoint initial_pos = route.getPositionAt(0.0);

        auto fill_start_user = [&](StartUserParams& user_params,
                                   unsigned int trajectory_id,
                                   unsigned int trajectory_type) {
            user_params.trajectory_id = trajectory_id;
            user_params.trajectory_type = trajectory_type;
            user_params.initial_user_pos_x = initial_pos.x;
            user_params.initial_user_pos_y = initial_pos.y;
            user_params.initial_user_pos_z = initial_pos.z;
        };

        fill_start_user(start_cmd.users[0],
                        static_cast<unsigned int>(config_.trajectory_ID),
                        static_cast<unsigned int>(config_.trajectory_type));
        fill_start_user(start_cmd.users[1],
                        static_cast<unsigned int>(config_.trajectory_ID_user2),
                        static_cast<unsigned int>(config_.trajectory_type_user2));

        std::cout << "准备向网络节点发送START命令..." << std::endl;
        bool send_result = udp_comm.send(&start_cmd, sizeof(start_cmd));
        if (send_result) {
            std::cout << "START命令发送成功" << std::endl;
        } else {
            std::cerr << "START命令发送失败" << std::endl;
        }
        return send_result;
    } catch (const std::exception& e) {
        std::cerr << "发送START命令时发生异常：" << e.what() << std::endl;
        return false;
    }
}

void TrainSimulator::run_simulation_non_blocking() {
    std::cout << "Simulation timer started. Running in the background." << std::endl;
    timer.start();
}

bool TrainSimulator::stop_simulation() {
    timer.stop();
    std::cout << "向网络节点发送STOP命令..." << std::endl;
    StopCommand stop_cmd = {};
    stop_cmd.command_word = 0x0ABC0003;
    bool result = udp_comm.send(&stop_cmd, sizeof(stop_cmd));
    std::cout << (result ? "STOP命令发送成功。" : "STOP命令发送失败。") << std::endl;
    return result;
}

void TrainSimulator::print_current_status() const {
    if (train_controller_ptr) {
        train_controller_ptr->printCurrentState();
    }
}

void TrainSimulator::set_control_mode(TrainController::ControlMode mode) {
    if (train_controller_ptr) {
        train_controller_ptr->setControlMode(mode);
    }
}

void TrainSimulator::set_control_level(TrainController::ControlLevel level) {
    if (train_controller_ptr) {
        train_controller_ptr->setControlLevel(level);
    }
}

double TrainSimulator::getCurrentSpeed() const {
    if (train_controller_ptr) {
        // 从 TrainController 获取 KinematicState
        // 然后从 KinematicState 结构体中返回速度
        return train_controller_ptr->getCurrentState().velocity;
    }
    // 如果控制器不存在，返回0.0
    return 0.0;
}

double TrainSimulator::getSimulationTime () const {
    // 从计时器获取已用时间（秒）
    double elapsed_seconds = timer.get_elapsed_time_sec ();

    // 将已用秒数转换为毫秒
    auto elapsed_milliseconds = static_cast<long long>(elapsed_seconds * 1000.0);

    // 将基准开始时间添加到已用时间
    long long current_simulation_time = simulation_start_time_ + elapsed_milliseconds;

    return static_cast<double>(current_simulation_time);
}

// 在 Simulator/TrainSimulator.cpp 文件末尾添加

const KinematicState& TrainSimulator::getCurrentState() const
{
    if (train_controller_ptr) {
        return train_controller_ptr->getCurrentState();
    }

    // 如果 train_controller_ptr 为空，返回一个安全的默认值
    // 这是一个好的防御性编程习惯
    static const KinematicState empty_state = {};
    return empty_state;
}



// 新增的函数实现
GeodeticPoint TrainSimulator::getCurrentPositionBLH() const {
    if (train_controller_ptr && route.isInitialized()) {
        // 1. 获取一维运动状态
        const KinematicState& state_1d = train_controller_ptr->getCurrentState();

        // 2. 从路径获取对应的 ECEF 坐标
        const ECEFPoint pos_3d_ecef = route.getPositionAt(state_1d.position);

        // 3. 将 ECEF 坐标转换为大地坐标并返回
        return GeoUtils::ecefToGeodetic(pos_3d_ecef);
    }

    // 如果无法获取，返回一个默认的零点坐标
    return GeodeticPoint{};
}
