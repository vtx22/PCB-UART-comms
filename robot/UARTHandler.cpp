#include "UARTHandler.hpp"

UARTHandler::UARTHandler(Robot *_robot) : robot(_robot)
{
    //O_RDWR - Open for reading and writing.
    uart0 = open("/dev/ttyS0", O_RDWR);
    if (uart0 == -1)
    {
        printf("UART error: unable to open\n");
    }

    //CLOCAL - Ignore modem status lines
    //CREAD - Enable receiver
    //IGNPAR = Ignore characters with parity errors
    struct termios options;
    tcgetattr(uart0, &options);
    options.c_cflag = B115200 | CS8 | CLOCAL | CREAD;
    options.c_iflag = IGNPAR;
    options.c_oflag = 0;
    options.c_lflag = 0;
    tcflush(uart0, TCIFLUSH);
    tcsetattr(uart0, TCSANOW, &options);
}

UARTHandler::~UARTHandler()
{
    close(uart0);
}

void UARTHandler::run()
{
    if (uart0 < 0)
    {
        std::cout << "UART error: cannot receive, no uart configured\n";
        return;
    }

    uint8_t rxBuffer[stmRcMessageLength];

    while (1)
    {
        int rxLength = read(uart0, (void *)rxBuffer, stmRcMessageLength);
        if (rxLength <= 0)
        {
            std::cout << "UART error: no bytes received\n";
            return;
        }

        if (rxBuffer[0] != primaryId)
        {
            std::cout << "UART error: wrong primary id " << toHexString(rxBuffer, rxLength) << "\n";
            return;
        }

        switch (rxBuffer[1])
        {
            case 0x01:
                interpretStatus(rxBuffer, rxLength);
                break;
            case 0x05:
                interpretBatteryStatus(rxBuffer, rxLength);
                break;
            case 0x0A:
                interpretMotorStatus(rxBuffer, rxLength);
                break;
            case 0x0F:
                interpretSensorStatus(rxBuffer, rxLength);
                break;
            default:
                std::cout << "UART error: invalid command id " << toHexString(rxBuffer, rxLength) << "\n";
                break;
        }
    }
}

void UARTHandler::interpretStatus(uint8_t *data, uint8_t length)
{
    if (length != 4)
    {
        std::cout << "UART error: invalid message length " << toHexString(data, length) << " (status)\n";
        return;
    }
    if (!checkChecksum(data, length))
    {
        std::cout << "UART error: invalid checksum " << toHexString(data, length) << " (status)\n";
        return;
    }
    if (robot->uartDebug)
    {
        std::cout << "UART receive: " << toHexString(data, length) << " (status)\n";
    }

    // TODO
    uint8_t code = data[2];
}

void UARTHandler::interpretBatteryStatus(uint8_t *data, uint8_t length)
{
    if (length != 7)
    {
        std::cout << "UART error: invalid message length " << toHexString(data, length) << " (battery status)\n";
        return;
    }
    if (!checkChecksum(data, length))
    {
        std::cout << "UART error: invalid checksum " << toHexString(data, length) << " (battery status)\n";
        return;
    }
    if (robot->uartDebug)
    {
        std::cout << "UART receive: " << toHexString(data, length) << " (battery status)\n";
    }

    int16_t volt, amps;
    memcpy(&volt, data + 2, sizeof(volt));
    memcpy(&amps, data + 4, sizeof(amps));

    BatteryData &bd = robot->getBatteryData();
    bd.lock();
    bd.volt = volt;
    bd.amps = amps;
    bd.unlock();
}

void UARTHandler::interpretMotorStatus(uint8_t *data, uint8_t length)
{
    if (length != 11)
    {
        std::cout << "UART error: invalid message length " << toHexString(data, length) << " (motor status)\n";
        return;
    }
    if (!checkChecksum(data, length))
    {
        std::cout << "UART error: invalid checksum " << toHexString(data, length) << " (motor status)\n";
        return;
    }
    if (robot->uartDebug)
    {
        std::cout << "UART receive: " << toHexString(data, length) << " (motor status)\n";
    }

    int16_t rpm0, rpm1, rpm2, rpm3;
    memcpy(&rpm0, data + 2, sizeof(rpm0));
    memcpy(&rpm1, data + 4, sizeof(rpm1));
    memcpy(&rpm2, data + 6, sizeof(rpm2));
    memcpy(&rpm3, data + 8, sizeof(rpm3));

    MotorData &md = robot->getMotorData();
    md.lock();
    md.rpm0 = rpm0;
    md.rpm1 = rpm1;
    md.rpm2 = rpm2;
    md.rpm3 = rpm3;
    md.unlock();
}

void UARTHandler::interpretSensorStatus(uint8_t *data, uint8_t length)
{
    if (length != 15)
    {
        std::cout << "UART error: invalid message length " << toHexString(data, length) << " (sensor status)\n";
        return;
    }
    if (!checkChecksum(data, length))
    {
        std::cout << "UART error: invalid checksum " << toHexString(data, length) << " (sensor status)\n";
        return;
    }
    if (robot->uartDebug)
    {
        std::cout << "UART receive: " << toHexString(data, length) << " (sensor status)\n";
    }

    int16_t sens0, sens1, sens2, sens3, sens4, sens5;
    memcpy(&sens0, data + 2, sizeof(sens0));
    memcpy(&sens1, data + 4, sizeof(sens1));
    memcpy(&sens2, data + 6, sizeof(sens2));
    memcpy(&sens3, data + 8, sizeof(sens3));
    memcpy(&sens4, data + 10, sizeof(sens4));
    memcpy(&sens5, data + 12, sizeof(sens5));

    SensorData &sd = robot->getSensorData();
    sd.lock();
    sd.sens0 = sens0;
    sd.sens1 = sens1;
    sd.sens2 = sens2;
    sd.sens3 = sens3;
    sd.sens4 = sens4;
    sd.sens5 = sens5;
    sd.unlock();
}

void UARTHandler::sendStatusData(bool generalEnable, bool sensorEnable)
{
    if (uart0 < 0)
    {
        std::cout << "UART error: cannot send, no uart configured\n";
        return;
    }

    uint8_t txBuffer[stmSdMessageLength] = {0};
    txBuffer[0] = primaryId;
    txBuffer[1] = 0x01;

    txBuffer[2] = generalEnable | (sensorEnable << 1);

    txBuffer[11] = putChecksum((uint8_t *)&txBuffer);

    if (write(uart0, &txBuffer[0], stmSdMessageLength) < 0)
    {
        std::cout << "UART error while sending status data " << toHexString(txBuffer, stmSdMessageLength) << "\n";
    }
    else if (robot->uartDebug)
    {
        std::cout << "UART send: " << toHexString(txBuffer, stmSdMessageLength) << " (status)\n";
    }
}

void UARTHandler::sendDriveData(DriveData &dd)
{
    if (uart0 < 0)
    {
        std::cout << "UART error: cannot send, no uart configured\n";
        return;
    }

    uint8_t txBuffer[stmSdMessageLength] = {0};
    txBuffer[0] = primaryId;
    txBuffer[1] = 0x08;

    dd.lock();
    memcpy(txBuffer + 2, &(dd.x), sizeof(dd.x));
    memcpy(txBuffer + 6, &(dd.y), sizeof(dd.y));
    txBuffer[10] = (dd.spd << 1) | dd.rot;
    dd.unlock();

    txBuffer[11] = putChecksum((uint8_t *)&txBuffer);

    if (write(uart0, &txBuffer[0], stmSdMessageLength) < 0)
    {
        std::cout << "UART error while sending drive data " << toHexString(txBuffer, stmSdMessageLength) << "\n";
    }
    else if (robot->uartDebug)
    {
        std::cout << "UART send: " << toHexString(txBuffer, stmSdMessageLength) << " (drive data)\n";
    }
}

uint8_t UARTHandler::putChecksum(uint8_t *data)
{
    uint8_t checksum = 0;
    for (uint8_t i = 0; i < stmSdMessageLength - 1; i++)
    {
        checksum ^= data[i];
    }
    return checksum;
}

bool UARTHandler::checkChecksum(uint8_t *data, uint8_t length)
{
    uint8_t checker = 0;
    for (uint8_t i = 0; i < length; i++)
    {
        checker ^= data[i];
    }
    return checker == 0;
}

std::string UARTHandler::toHexString(uint8_t *data, uint8_t length)
{
    std::stringstream ss;
    ss << "0x";
    for (uint8_t i = 0; i < length; i++)
    {
        ss << std::hex << (int)data[i];
    }
    return ss.str();
}
