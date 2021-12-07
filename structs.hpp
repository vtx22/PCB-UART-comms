#pragma once

#include <mutex>

struct DriveData : public std::mutex
{
    float x = 0.0;
    float y = 0.0;
    uint8_t rot = 0;
    uint8_t spd = 0;
};

struct MotorData : public std::mutex
{
    int16_t rpm0 = 0;
    int16_t rpm1 = 0;
    int16_t rpm2 = 0;
    int16_t rpm3 = 0;
};

struct BatteryData : public std::mutex
{
    int16_t volt = 0;
    int16_t amps = 0;
};

struct SensorData : public std::mutex
{
    int16_t sens0 = 0;
    int16_t sens1 = 0;
    int16_t sens2 = 0;
    int16_t sens3 = 0;
    int16_t sens4 = 0;
    int16_t sens5 = 0;
};
