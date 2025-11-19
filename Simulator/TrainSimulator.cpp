#include "TrainSimulator.h"
#include <iostream>
#include <functional>
#include <chrono>
#include <thread>
#include <stdexcept>
#include <algorithm>

namespace {
std::string narrow(const std::wstring& w) {
    return std::string(w.begin(), w.end());
}

double clamp_center(double s_center, double train_length, double route_length) {
    const double half_len = train_length * 0.5;
    const double min_s = half_len;
    const double max_s = std::max(half_len, route_length - half_len);
    return std::clamp(s_center, min_s, max_s);
}
} // namespace

TrainSimulator::TrainSimulator(const SimulatorConfiguration& config)
    : config_(config),
      udp_comm(narrow(config.ip), config.port),
      trajectory_sequence_numbers_{0, 0},
      simulation_start_time_(config.simulation_start_time) {

    std::cout << "正在使用配置构造TrainSimulator..." << std::endl;
    const std::string route_path = narrow(config_.route_file);
    if (!route.loadFromFile(route_path)) {
        throw std::runtime_error("加载路线文件失败: " + route_path);
    }
    std::cout << "路线加载成功。总距离：" << route.getTotalDistance() << "米" << std::endl;

    train_controller_ptr = std::make_unique<TrainController>(config_.test_vehicle, route.getTotalDistance());
    std::cout << "列车控制器初始化完成" << std::endl;

    timer.setInterval(config_.SIMULATION_INTERVAL_MS);
    timer.setCallback([this]() {
        const double dt = static_cast<double>(config_.SIMULATION_INTERVAL_MS) / 1000.0;

        train_controller_ptr->update(dt);
        const KinematicState state_1d = train_controller_ptr->getCurrentState();

        const double route_length = route.getTotalDistance();
        const double half_len = config_.test_vehicle.trainLong * 0.5;
        const double clamped_center_s = clamp_center(state_1d.position, config_.test_vehicle.trainLong, route_length);
        const bool clamped = clamped_center_s != state_1d.position;

        const double tangential_speed = clamped ? 0.0 : state_1d.velocity;
        const double tangential_acc = clamped ? 0.0 : state_1d.acceleration;
        const double tangential_jerk = clamped ? 0.0 : state_1d.jerk;

        const double s_head = std::min(route_length, clamped_center_s + half_len);
        const double s_tail = std::max(0.0, clamped_center_s - half_len);

        const unsigned long long seq_user1 = ++trajectory_sequence_numbers_[0];
        unsigned long long seq_user2 = 0;
        if (config_.enable_second_user) {
            seq_user2 = ++trajectory_sequence_numbers_[1];
        }

        auto fill_user_data = [&](TrajectoryData& user_data,
                                  unsigned int trajectory_id,
                                  unsigned int trajectory_type,
                                  unsigned long long seq_number,
                                  double s_sample) {
            user_data.trajectory_data_seq_num = seq_number;
            user_data.trajectory_time = (seq_number - 1) * dt;
            user_data.trajectory_id = trajectory_id;
            user_data.trajectory_type = trajectory_type;

            const ECEFPoint pos_3d = route.getPositionAt(s_sample);
            const ECEFPoint vel_3d = route.getVelocityAt(s_sample, tangential_speed);
            const ECEFPoint acc_3d = route.getAccelerationAt(s_sample, tangential_speed, tangential_acc);
            const ECEFPoint jerk_3d = route.getJerkAt(s_sample, tangential_speed, tangential_acc, tangential_jerk);

            user_data.user_pos_x = pos_3d.x; user_data.user_pos_y = pos_3d.y; user_data.user_pos_z = pos_3d.z;
            user_data.user_vel_x = vel_3d.x; user_data.user_vel_y = vel_3d.y; user_data.user_vel_z = vel_3d.z;
            user_data.user_acc_x = acc_3d.x; user_data.user_acc_y = acc_3d.y; user_data.user_acc_z = acc_3d.z;
            user_data.user_jerk_x = jerk_3d.x; user_data.user_jerk_y = jerk_3d.y; user_data.user_jerk_z = jerk_3d.z;
        };

        if (config_.enable_second_user) {
            DualTrajectoryData dual_packet{};
            fill_user_data(dual_packet.user1,
                           static_cast<unsigned int>(config_.trajectory_ID),
                           static_cast<unsigned int>(config_.trajectory_type),
                           seq_user1,
                           s_head);
            fill_user_data(dual_packet.user2,
                           static_cast<unsigned int>(config_.trajectory_ID_user2),
                           static_cast<unsigned int>(config_.trajectory_type_user2),
                           seq_user2,
                           s_tail);

            udp_comm.send(&dual_packet, sizeof(dual_packet));
        } else {
            TrajectoryData single_packet{};
            fill_user_data(single_packet,
                           static_cast<unsigned int>(config_.trajectory_ID),
                           static_cast<unsigned int>(config_.trajectory_type),
                           seq_user1,
                           s_head);
            udp_comm.send(&single_packet, sizeof(single_packet));
        }
    });
    std::cout << "TrainSimulator构造成功，回调已设置" << std::endl;
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

        StartCommand start_cmd{};
        start_cmd.command_word = config_.enable_second_user ? 0x000000000ABC0002ULL : 0x000000000ABC0001ULL;
        start_cmd.reserved_after_cmd = 0;
        start_cmd.simulation_duration = static_cast<uint64_t>(config_.simulation_duration);
        start_cmd.simulation_start_time = static_cast<uint64_t>(config_.simulation_start_time);

        const double route_length = route.getTotalDistance();
        const double half_len = config_.test_vehicle.trainLong * 0.5;
        const double s_center0 = clamp_center(0.0, config_.test_vehicle.trainLong, route_length);
        const double s_head0 = std::min(route_length, s_center0 + half_len);
        const double s_tail0 = std::max(0.0, s_center0 - half_len);

        const ECEFPoint pos_head = route.getPositionAt(s_head0);
        const ECEFPoint pos_tail = route.getPositionAt(s_tail0);

        auto fill_start_user = [](StartUserParams& user_params,
                                  unsigned int trajectory_id,
                                  unsigned int trajectory_type,
                                  const ECEFPoint& initial_pos) {
            user_params.trajectory_id = trajectory_id;
            user_params.trajectory_type = trajectory_type;
            user_params.initial_user_pos_x = initial_pos.x;
            user_params.initial_user_pos_y = initial_pos.y;
            user_params.initial_user_pos_z = initial_pos.z;
        };

        fill_start_user(start_cmd.users[0],
                        static_cast<unsigned int>(config_.trajectory_ID),
                        static_cast<unsigned int>(config_.trajectory_type),
                        pos_head);

        size_t payload_size = 32 + sizeof(StartUserParams); // 默认单用户

        if (config_.enable_second_user) {
            fill_start_user(start_cmd.users[1],
                            static_cast<unsigned int>(config_.trajectory_ID_user2),
                            static_cast<unsigned int>(config_.trajectory_type_user2),
                            pos_tail);
            payload_size = 32 + 2 * sizeof(StartUserParams);
        }

        start_cmd.data_length = static_cast<uint32_t>(payload_size);

        std::cout << "准备向网络节点发送START命令..." << std::endl;
        bool send_result = udp_comm.send(&start_cmd, payload_size);
        std::cout << (send_result ? "START命令发送成功" : "START命令发送失败") << std::endl;
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
    StopCommand stop_cmd{};
    stop_cmd.command_word = 0x000000000ABC0003ULL;
    stop_cmd.reserved_after_cmd = 0;
    stop_cmd.data_length = static_cast<uint32_t>(sizeof(StopCommand));
    bool result = udp_comm.send(&stop_cmd, sizeof(stop_cmd));
    std::cout << (result ? "STOP命令发送成功" : "STOP命令发送失败") << std::endl;
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
        return train_controller_ptr->getCurrentState().velocity;
    }
    return 0.0;
}

double TrainSimulator::getSimulationTime() const {
    double elapsed_seconds = timer.get_elapsed_time_sec();
    auto elapsed_milliseconds = static_cast<long long>(elapsed_seconds * 1000.0);
    long long current_simulation_time = simulation_start_time_ + elapsed_milliseconds;
    return static_cast<double>(current_simulation_time);
}

const KinematicState& TrainSimulator::getCurrentState() const {
    if (train_controller_ptr) {
        return train_controller_ptr->getCurrentState();
    }
    static const KinematicState empty_state = {};
    return empty_state;
}

GeodeticPoint TrainSimulator::getCurrentPositionBLH() const {
    if (train_controller_ptr && route.isInitialized()) {
        const KinematicState state_1d = train_controller_ptr->getCurrentState();
        const ECEFPoint pos_3d_ecef = route.getPositionAt(state_1d.position);
        return GeoUtils::ecefToGeodetic(pos_3d_ecef);
    }
    return GeodeticPoint{};
}
