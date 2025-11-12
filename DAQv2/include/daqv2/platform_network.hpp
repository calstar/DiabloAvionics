#pragma once

#include <cstdint>

// ====================== Platform networking includes ======================
#ifdef _WIN32
  #define _WINSOCK_DEPRECATED_NO_WARNINGS
  #include <winsock2.h>
  #include <ws2tcpip.h>
  #pragma comment(lib, "Ws2_32.lib")
  using socklen_t = int;
  using ssize_t   = int;

  static inline uint64_t htobe64(uint64_t x) {
    return ( (uint64_t)htonl((uint32_t)(x >> 32)) ) |
           ( (uint64_t)htonl((uint32_t)(x & 0xffffffff)) << 32 );
  }
  static inline uint64_t be64toh(uint64_t x) {
    return ( (uint64_t)ntohl((uint32_t)(x >> 32)) ) |
           ( (uint64_t)ntohl((uint32_t)(x & 0xffffffff)) << 32 );
  }

#elif defined(__APPLE__)
  #include <arpa/inet.h>
  #include <sys/socket.h>
  #include <netinet/in.h>
  #include <unistd.h>
  #include <libkern/OSByteOrder.h>
  #define closesocket close
  static inline uint64_t htobe64(uint64_t x){ return OSSwapHostToBigInt64(x); }
  static inline uint64_t be64toh(uint64_t x){ return OSSwapBigToHostInt64(x); }

#else // Linux and friends
  #include <arpa/inet.h>
  #include <sys/socket.h>
  #include <netinet/in.h>
  #include <unistd.h>
  #include <endian.h>
  #define closesocket close
#endif

