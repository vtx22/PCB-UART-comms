#include "BTHandler.hpp"

BTHandler::BTHandler(Robot *_robot) : robot(_robot)
{
    sock = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);

    sockaddr_rc loc_addr = {0};
    loc_addr.rc_family = AF_BLUETOOTH;
    loc_addr.rc_bdaddr = (bdaddr_t){{0, 0, 0, 0, 0, 0}}; //BDADDR_ANY
    loc_addr.rc_channel = 1;
    bind(sock, (sockaddr *)&loc_addr, sizeof(loc_addr));
}

void BTHandler::run()
{
    int bytes_read;
    uint8_t buf[16] = {0};

    while (1)
    {
        listen(sock, 1);

        printf("BT: listening on socket %d\n", sock.load());

        sockaddr_rc rem_addr = {0};
        unsigned int opt = sizeof(rem_addr);
        client = accept(sock, (sockaddr *)&rem_addr, &opt);

        ba2str(&rem_addr.rc_bdaddr, (char *)buf);
        printf("BT: accepted connection from %s\n", buf);
        memset(buf, 0, sizeof(buf));

        while (1)
        {
            bytes_read = recv(client, buf, btRcMessageLength, 0);
            if (bytes_read > 0)
            {
                interpretBytes(buf);
            }
            else
            {
                robot->setRobotMode(off);
                robot->setAutoMode(none);
                printf("BT: connection terminated!\n");
                break;
            }
        }
        close(client);
        client = 0;
    }

    close(sock);
    sock = 0;
}

void BTHandler::interpretBytes(uint8_t *data)
{
    switch (data[0])
    {
        case 0x01:
            interpretSignOnOff(data);
            break;
        case 0x11:
            interpretManualSet(data);
            break;
        case 0x12:
            interpretManualMove(data);
            break;
        case 0x21:
            interpretAutoSet(data);
            break;
        case 0x22:
            interpretAutoSendRoute(data);
            break;
        case 0x23:
            interpretAutoSendPoint(data);
            break;
        default:
            std::cout << "BT error: invalid command id " << toHexString(data, btRcMessageLength) << "\n";
            break;
    }
}

void BTHandler::interpretSignOnOff(uint8_t *data)
{
    if (robot->btDebug)
    {
        std::cout << "BT receive: " << toHexString(data, btRcMessageLength) << " (sign on/off)\n";
    }

    if (data[1] == 0x00)
    {
        if (robot->getRobotMode() != off)
        {
            robot->setRobotMode(off);
        }
        else
        {
            std::cout << "BT error: message not expected " << toHexString(data, btRcMessageLength) << " (sign on/off)\n";
        }
    }
    else if (data[1] == 0x01)
    {
        if (robot->getRobotMode() == off)
        {
            robot->setRobotMode(idle);
        }
        else
        {
            std::cout << "BT error: message not expected " << toHexString(data, btRcMessageLength) << " (sign on/off)\n";
        }
    }
    else
    {
        std::cout << "BT error: invalid command data " << toHexString(data, btRcMessageLength) << " (sign on/off)\n";
    }
}

void BTHandler::interpretManualSet(uint8_t *data)
{
    if (robot->btDebug)
    {
        std::cout << "BT receive: " << toHexString(data, btRcMessageLength) << " (manual on/off)\n";
    }

    if (data[1] == 0x00)
    {
        if (robot->getRobotMode() == manual)
        {
            robot->setRobotMode(idle);
        }
        else
        {
            std::cout << "BT error: message not expected " << toHexString(data, btRcMessageLength) << " (manual on/off)\n";
        }
    }
    else if (data[1] == 0x01)
    {
        if (robot->getRobotMode() == idle)
        {
            robot->setRobotMode(manual);
        }
        else
        {
            std::cout << "BT error: message not expected " << toHexString(data, btRcMessageLength) << " (manual on/off)\n";
        }
    }
    else
    {
        std::cout << "BT error: invalid command data " << toHexString(data, btRcMessageLength) << " (manual on/off)\n";
    }
}

void BTHandler::interpretManualMove(uint8_t *data)
{
    if (robot->btDebug)
    {
        std::cout << "BT receive: " << toHexString(data, btRcMessageLength) << " (manual move)\n";
    }

    if (robot->getRobotMode() == manual)
    {
        float vx, vy;

        uint8_t hx[] = {data[4], data[3], data[2], data[1]};
        memcpy(&vx, &hx, sizeof(vx));

        uint8_t hy[] = {data[8], data[7], data[6], data[5]};
        memcpy(&vy, &hy, sizeof(vy));

        uint8_t spdRot = data[9];

        DriveData &dd = robot->getDriveData();
        dd.lock();
        dd.x = vx;
        dd.y = vy;
        dd.rot = spdRot | 0x01;
        dd.spd = spdRot >> 1;
        dd.unlock();
    }
    else
    {
        std::cout << "BT error: message not expected " << toHexString(data, btRcMessageLength) << " (manual move)\n";
    }
}

void BTHandler::interpretAutoSet(uint8_t *data)
{
    if (robot->btDebug)
    {
        std::cout << "BT receive: " << toHexString(data, btRcMessageLength) << " (auto set)\n";
    }

    if (data[1] == 0x00)
    {
        if (robot->getRobotMode() == automatic && robot->getAutoMode() != none)
        {
            robot->setAutoMode(none);
            robot->setRobotMode(idle);
        }
        else
        {
            std::cout << "BT error: message not expected " << toHexString(data, btRcMessageLength) << " (auto set)\n";
        }
    }
    else if (data[1] == 0x01)
    {
        if (robot->getRobotMode() != automatic && robot->getAutoMode() == none)
        {
            robot->setAutoMode(follow);
            robot->setRobotMode(automatic);
        }
        else
        {
            std::cout << "BT error: message not expected " << toHexString(data, btRcMessageLength) << " (auto set)\n";
        }
    }
    else if (data[1] == 0x02)
    {
        if (robot->getRobotMode() != automatic && robot->getAutoMode() == none)
        {
            robot->setAutoMode(route);
            robot->setRobotMode(automatic);
        }
        else
        {
            std::cout << "BT error: message not expected " << toHexString(data, btRcMessageLength) << " (auto set)\n";
        }
    }
    else if (data[1] == 0x03)
    {
        if (robot->getRobotMode() != automatic && robot->getAutoMode() == none)
        {
            robot->setAutoMode(learn);
            robot->setRobotMode(automatic);
        }
        else
        {
            std::cout << "BT error: message not expected " << toHexString(data, btRcMessageLength) << " (auto set)\n";
        }
    }
    else
    {
        std::cout << "BT error: invalid command data " << toHexString(data, btRcMessageLength) << " (auto set mode)\n";
    }
}

void BTHandler::interpretAutoSendRoute(uint8_t *data)
{
    if (robot->btDebug)
    {
        std::cout << "BT receive: " << toHexString(data, btRcMessageLength) << " (auto send route)\n";
    }

    //TODO
    if (data[1] == 0x00)
    {
        printf("auto send route end\n");
    }
    else if (data[1] == 0x01)
    {
        printf("auto send route start\n");
    }
    else
    {
        std::cout << "BT error: invalid command data " << toHexString(data, btRcMessageLength) << " (auto send route)\n";
    }
}

void BTHandler::interpretAutoSendPoint(uint8_t *data)
{
    if (robot->btDebug)
    {
        std::cout << "BT receive: " << toHexString(data, btRcMessageLength) << " (auto send point)\n";
    }

    //TODO
    uint8_t pid = data[1];
    uint8_t connections[8];
    memcpy(connections, data + 2, 8);

    printf("checkpoint: %d", pid);
    for (uint8_t i = 0; i < 8; i++)
    {
        printf(" %d", connections[i]);
    }
    printf("\n");
}

void BTHandler::sendStatusData(MotorData &md, BatteryData &bd)
{
    if (client <= 0)
    {
        std::cout << "BT error: cannot send, no client connected\n";
        return;
    }

    uint8_t bytes[btSdMessageLength] = {0};
    bytes[0] = 0x0A;

    md.lock();
    memcpy(bytes + 1, &(md.rpm0), sizeof(md.rpm0));
    memcpy(bytes + 3, &(md.rpm1), sizeof(md.rpm1));
    memcpy(bytes + 5, &(md.rpm2), sizeof(md.rpm2));
    memcpy(bytes + 7, &(md.rpm3), sizeof(md.rpm3));
    md.unlock();

    bd.lock();
    memcpy(bytes + 9, &(bd.volt), sizeof(bd.volt));
    memcpy(bytes + 11, &(bd.amps), sizeof(bd.amps));
    bd.unlock();

    if (send(client, bytes, sizeof(bytes), 0) <= 0)
    {
        std::cout << "BT error: error while sending status data\n";
    }
    else if (robot->btDebug)
    {
        std::cout << "BT send: " << toHexString(bytes, btSdMessageLength) << " (status data)\n";
    }
}

void BTHandler::sendSensorData(SensorData &sd)
{
    if (client <= 0)
    {
        std::cout << "BT error: cannot send, no client connected\n";
        return;
    }

    uint8_t bytes[btSdMessageLength] = {0};
    bytes[0] = 0x0B;

    sd.lock();
    memcpy(bytes + 1, &(sd.sens0), sizeof(sd.sens0));
    memcpy(bytes + 3, &(sd.sens1), sizeof(sd.sens1));
    memcpy(bytes + 5, &(sd.sens2), sizeof(sd.sens2));
    memcpy(bytes + 7, &(sd.sens3), sizeof(sd.sens3));
    memcpy(bytes + 9, &(sd.sens4), sizeof(sd.sens4));
    memcpy(bytes + 11, &(sd.sens5), sizeof(sd.sens5));
    sd.unlock();

    if (send(client, bytes, sizeof(bytes), 0) <= 0)
    {
        std::cout << "BT error: error while sending sensor data\n";
    }
    else if (robot->btDebug)
    {
        std::cout << "BT send: " << toHexString(bytes, btSdMessageLength) << " (sensor data)\n";
    }
}

void BTHandler::sendAutoLearnStatus(AutoMode mode)
{
    if (client <= 0)
    {
        std::cout << "BT error: cannot send, no client connected\n";
        return;
    }

    uint8_t bytes[btSdMessageLength] = {0};
    bytes[0] = 0x10;
    bytes[1] = (uint8_t)(mode - learn);

    if (send(client, bytes, sizeof(bytes), 0) <= 0)
    {
        std::cout << "BT error: error while sending auto learn status\n";
    }
    else if (robot->btDebug)
    {
        std::cout << "BT send: " << toHexString(bytes, btSdMessageLength) << " (auto learn status)\n";
    }
}

std::string BTHandler::toHexString(uint8_t *data, uint8_t length)
{
    std::stringstream ss;
    ss << "0x";
    for (uint8_t i = 0; i < length; i++)
    {
        ss << std::hex << (int)data[i];
    }
    return ss.str();
}
