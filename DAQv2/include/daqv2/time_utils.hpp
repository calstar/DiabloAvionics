#pragma once

#include <chrono>
#include <cstdint>

inline uint64_t now_us() {
    using namespace std::chrono;
    return (uint64_t)std::chrono::duration_cast<std::chrono::microseconds>(
        steady_clock::now().time_since_epoch()).count();
}

