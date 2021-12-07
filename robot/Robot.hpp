#pragma once

#include <chrono>
#include <iostream>
#include <thread>

class Robot;

#include "BTHandler.hpp"
#include "UARTHandler.hpp"
#include "enums.hpp"
#include "structs.hpp"

class Robot
{
public:
    const bool btDebug;
    const bool uartDebug;
    const bool stateDebug;

private:
    BTHandler *btHandler;
    UARTHandler *uartHandler;

    std::atomic<RobotMode> mode{off};
    std::atomic<AutoMode> autoMode{none};

    DriveData driveData;
    MotorData motorData;
    BatteryData batData;
    SensorData sensData;

    RobotMode lastMode = off;
    RobotMode thisMode = off;

    AutoMode lastAutoMode = none;
    AutoMode thisAutoMode = none;

    const std::chrono::duration<int, std::milli> uartInterval;
    const std::chrono::duration<int, std::milli> btInterval;

    const std::chrono::duration<int, std::milli> autoLearnStepTime;
    const int8_t autoLearnSteps = 10;
    int8_t autoLearnStep = 0;
    int16_t *autoLearnData = nullptr;
    int16_t autoLowThreshold = 0;
    int16_t autoHighThreshold = 0;

    std::chrono::_V2::system_clock::time_point currentTime;
    std::chrono::_V2::system_clock::time_point uartTimer;
    std::chrono::_V2::system_clock::time_point btTimer;
    std::chrono::_V2::system_clock::time_point autoTimer;

public:
    Robot(bool _btDebug, bool _uartDebug, bool _stateDebug);
    ~Robot();
    void run();

    void checkModeChanges();
    void checkAutoModeChanges();
    void checkSendUartUpdates();
    void checkSendBtUpdates();
    void checkAutoActions();

    RobotMode getRobotMode();
    void setRobotMode(RobotMode _mode);

    AutoMode getAutoMode();
    void setAutoMode(AutoMode _mode);

    DriveData &getDriveData();
    void resetDriveData();

    MotorData &getMotorData();
    BatteryData &getBatteryData();
    SensorData &getSensorData();
};
