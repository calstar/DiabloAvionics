#include "daqv2/udp_io.hpp"

#include <iostream>

bool UdpTx::open(const char* ip, uint16_t port) {
#ifdef _WIN32
    WSADATA w; if (WSAStartup(MAKEWORD(2,2), &w) != 0) return false;
#endif
    fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) return false;
    int enable = 1;
    setsockopt(fd, SOL_SOCKET, SO_BROADCAST, (char*)&enable, sizeof(enable));
    dst.sin_family = AF_INET;
    dst.sin_port   = htons(port);
    if (inet_pton(AF_INET, ip, &dst.sin_addr) != 1) return false;
    return true;
}

bool UdpTx::send(const std::vector<uint8_t>& pkt) {
    ssize_t n = ::sendto(fd, (const char*)pkt.data(), (int)pkt.size(), 0,
                         (sockaddr*)&dst, sizeof(dst));
    return n == (ssize_t)pkt.size();
}

UdpTx::~UdpTx(){
    if (fd>=0) closesocket(fd);
#ifdef _WIN32
    WSACleanup();
#endif
}

bool UdpRx::bind_any(uint16_t port) {
#ifdef _WIN32
    WSADATA w; if (WSAStartup(MAKEWORD(2,2), &w) != 0) return false;
#endif
    fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) return false;
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(port);
    return ::bind(fd, (sockaddr*)&addr, sizeof(addr)) == 0;
}

ssize_t UdpRx::recv(uint8_t* buf, size_t cap) {
    return ::recv(fd, (char*)buf, (int)cap, 0);
}

UdpRx::~UdpRx(){
    if (fd>=0) closesocket(fd);
#ifdef _WIN32
    WSACleanup();
#endif
}
