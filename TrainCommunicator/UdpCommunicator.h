#pragma once

#include <string>

/**
 * @brief 一个用于发送UDP数据包的通信器类。
 *
 * 该类封装了底层的socket编程细节，提供了一个简单的send接口。
 * 它会自动处理WSA的初始化与清理，并在析构时关闭socket。
 * 该类会自动为您发送的数据包添加一个通用的网络帧头。
 */
class UdpCommunicator {
public:
    /**
     * @brief 构造函数，初始化一个UDP发送器。
     * @param ip 目标服务器的IP地址。
     * @param port 目标服务器的端口号。
     * @throws std::runtime_error 如果初始化失败（例如，socket创建失败）。
     */
    UdpCommunicator(const std::string& ip, int port);

    /**
     * @brief 析构函数，自动关闭socket并清理资源。
     */
    ~UdpCommunicator();

    /**
     * @brief 发送一段二进制数据。
     * @param payload 指向要发送的数据（载荷）的指针。
     * @param payload_size 要发送的数据（载荷）的字节大小。
     * @return true 如果发送成功。
     * @return false 如果发送失败。
     */
    bool send(const void* payload, size_t payload_size);

    bool is_initialized() const;

    // --- 禁止拷贝构造和赋值，以防止socket资源管理混乱 ---
    UdpCommunicator(const UdpCommunicator&) = delete;
    UdpCommunicator& operator=(const UdpCommunicator&) = delete;

private:
    // 使用PImpl模式（指向实现的指针）来完全隐藏内部成员，
    // 使该头文件无需包含任何平台相关的头文件。
    class UdpImpl;
    UdpImpl* pImpl;
};