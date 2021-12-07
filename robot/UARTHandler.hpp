#pragma once

#include <fcntl.h>
#include <iostream>
#include <sstream>
#include <termios.h>
#include <unistd.h>

class UARTHandler;

#include "Robot.hpp"
#include "enums.hpp"
#include "structs.hpp"

class UARTHandler
{
private:
    const uint8_t stmRcMessageLength = 16;
    const uint8_t stmSdMessageLength = 12;
    const uint8_t primaryId = 0x69;

    std::atomic<int> uart0{-1};

    Robot *robot;

public:
    UARTHandler(Robot *_robot);
    ~UARTHandler();
    void run();

    void sendStatusData(bool generalEnable, bool sensorEnable);
    void sendDriveData(DriveData &dd);

private:
    void interpretStatus(uint8_t *data, uint8_t length);
    void interpretBatteryStatus(uint8_t *data, uint8_t length);
    void interpretMotorStatus(uint8_t *data, uint8_t length);
    void interpretSensorStatus(uint8_t *data, uint8_t length);

    uint8_t putChecksum(uint8_t *data);
    bool checkChecksum(uint8_t *data, uint8_t length);
    std::string toHexString(uint8_t *data, uint8_t length);
};