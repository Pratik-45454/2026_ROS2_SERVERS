#ifndef ROBOTLIBPC_TIME_HPP_
#define ROBOTLIBPC_TIME_HPP_

#include <chrono>
#include <cstdint>

inline uint32_t get_tick_ms()
{
    return static_cast<uint32_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());
}

inline uint64_t get_tick_us()
{
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());
}

#endif