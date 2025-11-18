#include "MillisecondTimer.h"
#include <iostream>

MillisecondTimer::MillisecondTimer()
    : running_(false), intervalTicks_(0), clockFrequency_(0), startTime_(0) {
    if (!QueryPerformanceFrequency(reinterpret_cast<LARGE_INTEGER*>(&clockFrequency_))) {
        throw std::runtime_error("High-resolution performance counter not supported.");
    }
}

MillisecondTimer::~MillisecondTimer() {
    stop();
}

void MillisecondTimer::setCallback(std::function<void()> callback) {
    tickCallback_ = callback;
}

void MillisecondTimer::setInterval(int ms) {
    intervalTicks_ = static_cast<long long>((static_cast<double>(ms) * clockFrequency_) / 1000.0);
}

void MillisecondTimer::start() {
    if (running_) {
        return;
    }
    running_ = true;

    long long current_ticks;
    QueryPerformanceCounter(reinterpret_cast<LARGE_INTEGER*>(&current_ticks));
    startTime_.store(current_ticks);

    if (timerThread_.joinable()) {
        timerThread_.join();
    }
    timerThread_ = std::thread(&MillisecondTimer::threadProc, this);
    SetThreadPriority(timerThread_.native_handle(), THREAD_PRIORITY_HIGHEST);
}

void MillisecondTimer::stop() {
    running_ = false;
    startTime_.store(0);
    if (timerThread_.joinable()) {
        timerThread_.join();
    }
}

double MillisecondTimer::get_elapsed_time_sec() const {
    if (!running_.load()) {
        return 0.0;
    }

    long long current_ticks;
    QueryPerformanceCounter(reinterpret_cast<LARGE_INTEGER*>(&current_ticks));

    long long start_ticks = startTime_.load();
    long long elapsed_ticks = current_ticks - start_ticks;

    return static_cast<double>(elapsed_ticks) / clockFrequency_;
}

void MillisecondTimer::threadProc() {
    long long currentTime = 0;
    QueryPerformanceCounter(reinterpret_cast<LARGE_INTEGER*>(&currentTime));
    long long nextTriggerTime = currentTime + intervalTicks_;

    while (running_) {
        while (currentTime < nextTriggerTime) {
            QueryPerformanceCounter(reinterpret_cast<LARGE_INTEGER*>(&currentTime));
        }
        nextTriggerTime = currentTime + intervalTicks_;
        if (tickCallback_) {
            tickCallback_();
        }
    }
}