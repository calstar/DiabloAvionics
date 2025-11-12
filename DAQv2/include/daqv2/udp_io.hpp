#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "daqv2/platform_network.hpp"

class UdpTx {
    int fd = -1;
    sockaddr_in dst{};
public:
    bool open(const char* ip, uint16_t port);
    bool send(const std::vector<uint8_t>& pkt);
    ~UdpTx();
};

class UdpRx {
    int fd = -1;
public:
    bool bind_any(uint16_t port);
    ssize_t recv(uint8_t* buf, size_t cap);
    ~UdpRx();
};
