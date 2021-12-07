#pragma once

#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <iostream>
#include <sstream>

class BTHandler;

#include "Robot.hpp"
#include "enums.hpp"
#include "structs.hpp"

class BTHandler
{
private:
    const int btRcMessageLength = 10;
    const int btSdMessageLength = 16;

    std::atomic<int> sock{0};
    std::atomic<int> client{0};

    Robot *robot;

public:
    BTHandler(Robot *_robot);
    void run();

    void sendStatusData(MotorData &md, BatteryData &bd);
    void sendSensorData(SensorData &sd);
    void sendAutoLearnStatus(AutoMode mode);

private:
    void interpretBytes(uint8_t *data);
    void interpretSignOnOff(uint8_t *data);
    void interpretManualSet(uint8_t *data);
    void interpretManualMove(uint8_t *data);
    void interpretAutoSet(uint8_t *data);
    void interpretAutoSendRoute(uint8_t *data);
    void interpretAutoSendPoint(uint8_t *data);

    std::string toHexString(uint8_t *data, uint8_t length);
};