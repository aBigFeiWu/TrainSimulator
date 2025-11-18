#ifndef MILLISECONDTIMER_H
#define MILLISECONDTIMER_H
#pragma once

#include <windows.h>
#include <functional>
#include <thread>
#include <atomic>
#include <stdexcept>

class MillisecondTimer {
public:
    MillisecondTimer();
    ~MillisecondTimer();
    void setCallback(std::function<void()> callback);
    void setInterval(int ms);
    void start();
    void stop();

    /**
     * @brief 获取从定时器启动开始到当前的总运行时间。
     * @return 运行时间（秒）。如果定时器未运行，则返回0。
     */
    double get_elapsed_time_sec() const;

private:
    void threadProc();
    std::thread timerThread_;
    std::atomic<bool> running_;
    long long clockFrequency_;
    long long intervalTicks_;
    std::function<void()> tickCallback_;

    // 用于存储定时器启动时刻的滴答数，使用原子类型保证线程安全
    std::atomic<long long> startTime_; 
};
#endif //MILLISECONDTIMER_H