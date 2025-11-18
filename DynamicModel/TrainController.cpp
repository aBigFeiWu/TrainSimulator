#include "TrainController.h"
#include <chrono>
#include <thread>
#include <stdexcept>
#include <iomanip>
#include <cmath>

TrainController::TrainController(const TrainInfo& train_info, double track_length)
    : constraints_(create_constraints_from_info(train_info)),
      planner_(constraints_),
      track_length_(track_length)
{
    station_position_ = track_length_;
    std::cout << "1D Simulation configured. Track length: " << track_length_ << "m, Target station: " << station_position_ << "m." << std::endl;
    std::cout << "Train Constraints: MaxVel=" << constraints_.max_velocity << " m/s, MaxAccel=" << constraints_.max_acceleration << " m/s^2, MinAccel=" << constraints_.min_acceleration << " m/s^2" << std::endl;
    std::cout << "Jerk Gears (Low/Mid/High): " << constraints_.max_jerk[0] << "/" << constraints_.max_jerk[1] << "/" << constraints_.max_jerk[2] << " m/s^3" << std::endl;

    output_file_.open("simulation_log.dat");
    if (output_file_.is_open()) {
        output_file_ << "# 1:Distance(m) 2:Vel(m/s) 3:Accel(m/s^2) 4:Jerk(m/s^3)\n";
        std::cout << "Log file 'simulation_log.dat' has been created." << std::endl;
    }
}

TrainController::~TrainController() {
    if (output_file_.is_open()) {
        output_file_.close();
        std::cout << "Log file 'simulation_log.dat' has been saved and closed." << std::endl;
    }
}

void TrainController::setControlMode(ControlMode mode) {
    m_control_mode = mode;
    if (mode == ControlMode::AUTOMATIC) {
        std::cout << "Switched to AUTOMATIC mode." << std::endl;
        train_state_ = TrainState::STOPPED; // 重置自动模式状态
        planner_.setTargetVelocity(0.0); // 确保目标速度为0
    } else {
        std::cout << "Switched to MANUAL mode." << std::endl;
    }
}

void TrainController::setControlLevel(ControlLevel level) {
    if (m_control_mode == ControlMode::MANUAL) {
        m_current_level = level;
        // std::cout << "Manual control level set." << std::endl; // 可以取消注释以获得更详细的输出
    } else {
        std::cerr << "Warning: Can't set control level in AUTOMATIC mode." << std::endl;
    }
}

void TrainController::update(double dt) {
    switch (m_control_mode) {
        case ControlMode::AUTOMATIC:
            update_state_machine();
            planner_.update_for_velocity(dt, state_);
            break;

        case ControlMode::MANUAL:
        { // 使用花括号创建一个作用域
            double target_accel = 0.0;
            switch (m_current_level) {
                case ControlLevel::IDLE:       target_accel = 0.0; break;
                case ControlLevel::CRUISE:     target_accel = 0.0; break;
                case ControlLevel::TRACTION_1: target_accel = constraints_.max_acceleration * (1.0 / 3.0); break;
                case ControlLevel::TRACTION_2: target_accel = constraints_.max_acceleration * (2.0 / 3.0); break;
                case ControlLevel::TRACTION_3: target_accel = constraints_.max_acceleration; break;
                case ControlLevel::BRAKE_1:    target_accel = constraints_.min_acceleration * (1.0 / 3.0); break;
                case ControlLevel::BRAKE_2:    target_accel = constraints_.min_acceleration * (2.0 / 3.0); break;
                case ControlLevel::BRAKE_3:    target_accel = constraints_.min_acceleration; break;
            }

            // --- 新增的修复逻辑 ---
            // 如果当前速度为0或更小，并且目标加速度为负（即正在制动），
            // 那么就强制将目标加速度设为0，防止列车后退。
            if (state_.velocity <= 1e-6 && target_accel < 0.0) { // 使用一个小的阈值1e-6防止浮点数误差
                target_accel = 0.0;
            }

            planner_.setTargetAcceleration(target_accel);
            planner_.update_for_acceleration(dt, state_);
        }
            break;
    }

    print_state();

    if (train_state_ == TrainState::STOPPED && state_.position > 1.0) {
        state_.position = station_position_;
        state_.velocity = 0.0;
        state_.acceleration = 0.0;
        state_.jerk = 0.0;
    }
}

void TrainController::update_state_machine() {
    double distance_to_station = station_position_ - state_.position;
    switch (train_state_) {
        case TrainState::STOPPED:
            std::cout << "\nTrain is starting...\n";
            train_state_ = TrainState::ACCELERATING;
            planner_.setTargetVelocity(constraints_.max_velocity);
            break;
        case TrainState::ACCELERATING:
            if (std::abs(state_.velocity - constraints_.max_velocity) < 0.1) {
                std::cout << "\nReached max velocity. Now cruising.\n";
                train_state_ = TrainState::CRUISING;
            }
            break;
        case TrainState::CRUISING:
            if (distance_to_station < COASTING_BUFFER_DISTANCE) {
                std::cout << "\nEntering coasting buffer. Preparing to stop.\n";
                train_state_ = TrainState::COASTING;
                planner_.setTargetVelocity(0.0);
            }
            break;
        case TrainState::COASTING:
            if (state_.velocity < 0.1 && distance_to_station < 1.0) {
                 std::cout << "\nTrain has stopped at the station.\n";
                train_state_ = TrainState::STOPPED;
            }
            break;
        case TrainState::BRAKING:
            break;
    }
}

MotionConstraints TrainController::create_constraints_from_info(const TrainInfo& train_info) {
    MotionConstraints constraints;
    constraints.max_velocity = train_info.maxSpeed / 3.6;
    constraints.max_acceleration = train_info.tractionAcceleration;
    constraints.min_acceleration = train_info.brakingAcceleration;
    double high_gear_jerk = (constraints.max_acceleration - constraints.min_acceleration) / 2.0;
    constraints.max_jerk[2] = high_gear_jerk;
    constraints.max_jerk[1] = high_gear_jerk * 2.0 / 3.0;
    constraints.max_jerk[0] = high_gear_jerk * 1.0 / 3.0;
    return constraints;
}

void TrainController::print_state() const {
    printf("Pos: %9.2fm | Vel: %5.2fm/s | Accel: %4.2fm/s^2 | Jerk: %4.2fm/s^3\n",
           state_.position, state_.velocity, state_.acceleration, state_.jerk);
    if (output_file_.is_open()) {
        output_file_ << std::fixed << std::setprecision(4)
                     << state_.position << " "
                     << state_.velocity << " "
                     << state_.acceleration << " "
                     << state_.jerk << "\n";
    }
}

void TrainController::printCurrentState() const {
    print_state();
}

const KinematicState& TrainController::getCurrentState() const {
    return state_;
}