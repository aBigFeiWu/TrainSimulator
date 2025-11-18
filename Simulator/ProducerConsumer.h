#ifndef PRODUCERCONSUMER_H
#define PRODUCERCONSUMER_H

#include "TrainCommunicator/MillisecondTimer.h" // 包含您提供的定时器头文件
#include <functional>
#include <thread>
#include <atomic>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <iostream>

/**
 * @brief 一个线程安全的队列模板类
 * @tparam T 队列中存储的数据类型
 */
template<typename T>
class ThreadSafeQueue {
public:
    ThreadSafeQueue() = default;
    ~ThreadSafeQueue() {
        stop();
    }

    // 向队列中推送一个元素
    void push(T value) {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_queue.push(std::move(value));
        m_cond.notify_one();
    }

    // 从队列中弹出一个元素。如果队列被停止且为空，则返回false。
    // 这是一个阻塞操作，直到有元素可用或队列被停止。
    bool pop(T& value) {
        std::unique_lock<std::mutex> lock(m_mutex);
        m_cond.wait(lock, [this] { return !m_queue.empty() || m_stopped; });

        if (m_stopped && m_queue.empty()) {
            return false;
        }

        value = std::move(m_queue.front());
        m_queue.pop();
        return true;
    }

    // 停止队列，唤醒所有等待的线程
    void stop() {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_stopped = true;
        m_cond.notify_all();
    }

    // 重置队列状态以便重新启动
    void reset() {
        std::lock_guard<std::mutex> lock(m_mutex);
        std::queue<T> empty;
        m_queue.swap(empty); // 清空队列
        m_stopped = false;
    }

private:
    std::queue<T> m_queue;
    std::mutex m_mutex;
    std::condition_variable m_cond;
    std::atomic<bool> m_stopped{false};
};


/**
 * @brief 将生产者和消费者打包在一起的类
 * @tparam T 在生产者和消费者之间传递的数据类型
 */
template<typename T>
class ProducerConsumer {
public:
    /**
     * @param consumer_task 用户定义的消费者任务，接收一个参数 T。
     */
    explicit ProducerConsumer(std::function<void(T)> consumer_task)
        : m_consumer_task(std::move(consumer_task)) {}

    ~ProducerConsumer() {
        stop();
    }

    // 禁止拷贝和赋值
    ProducerConsumer(const ProducerConsumer&) = delete;
    ProducerConsumer& operator=(const ProducerConsumer&) = delete;

    /**
     * @brief 启动生产者定时器和消费者线程
     * @param producer_interval_ms 生产者的生产间隔（毫秒）
     * @param producer_task 用户定义的生产者任务，返回类型为 T 的数据
     */
    void start(int producer_interval_ms, std::function<T()> producer_task) {
        if (m_is_running) {
            std::cout << "ProducerConsumer is already running." << std::endl;
            return;
        }

        m_queue.reset();
        m_is_running = true;

        // 设置并启动生产者定时器
        m_producer_timer.setInterval(producer_interval_ms);
        m_producer_timer.setCallback([this, producer_task]() {
            if (m_is_running) {
                m_queue.push(producer_task());
            }
        });
        m_producer_timer.start();

        // 启动消费者线程
        m_consumer_thread = std::thread(&ProducerConsumer::consumer_loop, this);
    }

    /**
     * @brief 优雅地停止生产者和消费者
     */
    void stop() {
        if (!m_is_running) {
            return;
        }

        m_is_running = false;
        m_producer_timer.stop();
        m_queue.stop(); // 唤醒可能正在等待的消费者线程

        if (m_consumer_thread.joinable()) {
            m_consumer_thread.join();
        }
        std::cout << "ProducerConsumer stopped." << std::endl;
    }

private:
    // 消费者线程的循环函数
    void consumer_loop() {
        std::cout << "Consumer thread started." << std::endl;
        while (true) {
            T data;
            // pop会阻塞，直到有数据或队列被停止
            if (!m_queue.pop(data)) {
                // 如果pop返回false，说明队列已停止且为空，可以安全退出
                break;
            }
            // 执行用户定义的回调
            m_consumer_task(data);
        }
        std::cout << "Consumer thread finished." << std::endl;
    }

    MillisecondTimer m_producer_timer;
    ThreadSafeQueue<T> m_queue;
    std::thread m_consumer_thread;
    std::function<void(T)> m_consumer_task;
    std::atomic<bool> m_is_running{false};
};

#endif //PRODUCERCONSUMER_H