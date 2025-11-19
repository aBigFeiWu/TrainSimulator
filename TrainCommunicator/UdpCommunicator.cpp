//
// Created by fyh on 25-7-24.
//
#include "UdpCommunicator.h"
#include "Protocol.h"
#include <winsock2.h>
#include <ws2tcpip.h>
#include <stdexcept>
#include <vector>
#include <cstring>

// 链接 Winsock
#pragma comment(lib, "ws2_32.lib")

// PImpl 模式的实现
class UdpCommunicator::UdpImpl {
public:
    SOCKET sock;
    sockaddr_in server_addr;

    UdpImpl() : sock(INVALID_SOCKET) {
        // 初始化 Winsock
        WSADATA wsaData;
        if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
            throw std::runtime_error("WSAStartup failed.");
        }
    }

    ~UdpImpl() {
        if (sock != INVALID_SOCKET) {
            closesocket(sock);
        }
        WSACleanup();
    }
};

UdpCommunicator::UdpCommunicator(const std::string& ip, int port) : pImpl(new UdpImpl()) {
    pImpl->sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (pImpl->sock == INVALID_SOCKET) {
        delete pImpl;
        throw std::runtime_error("Socket creation failed.");
    }

    memset(&pImpl->server_addr, 0, sizeof(pImpl->server_addr));
    pImpl->server_addr.sin_family = AF_INET;
    pImpl->server_addr.sin_port = htons(port);
    // 将IP地址字符串转换为网络格式
    if (inet_pton(AF_INET, ip.c_str(), &pImpl->server_addr.sin_addr) <= 0) {
        delete pImpl;
        throw std::runtime_error("Invalid address/ Address not supported.");
    }
}

UdpCommunicator::~UdpCommunicator() {
    delete pImpl;
}

bool UdpCommunicator::send(const void* payload, size_t payload_size) {
    if (pImpl->sock == INVALID_SOCKET) {
        return false;
    }

    // 1. 准备帧头
    NetworkFrameHeader header{};
    header.frame_flag = 0xA5A56666; // 固定的帧头标识
    header.frame_number = 0;        // 协议要求固定为 0
    header.frame_length = static_cast<uint32_t>(payload_size);
    header.reserved = 0;

    // 2. 组合帧头和载荷
    std::vector<char> packet(sizeof(NetworkFrameHeader) + payload_size);
    memcpy(packet.data(), &header, sizeof(NetworkFrameHeader));
    memcpy(packet.data() + sizeof(NetworkFrameHeader), payload, payload_size);

    // 3. 发送数据包
    int bytes_sent = sendto(pImpl->sock, packet.data(), static_cast<int>(packet.size()), 0,
                            (const sockaddr*)&pImpl->server_addr, sizeof(pImpl->server_addr));

    return bytes_sent == static_cast<int>(packet.size());
}

bool UdpCommunicator::is_initialized() const {
    // 成功初始化的主要标志：pImpl 有效且 socket 句柄有效
    return pImpl != nullptr && pImpl->sock != INVALID_SOCKET;
}
