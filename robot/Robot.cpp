#include "Robot.hpp"

Robot::Robot(bool _btDebug, bool _uartDebug, bool _stateDebug) : btDebug(_btDebug), uartDebug(_uartDebug), stateDebug(_stateDebug),
                                                                 uartInterval(100), btInterval(250), autoLearnStepTime(2500)
{
    if (btDebug)
    {
        std::cout << "running with BT debug\n";
    }
    if (uartDebug)
    {
        std::cout << "running with UART debug\n";
    }
    if (stateDebug)
    {
        std::cout << "running with state debug\n";
    }

    btHandler = new BTHandler(this);
    uartHandler = new UARTHandler(this);

    uartTimer = std::chrono::high_resolution_clock::now();
    btTimer = std::chrono::high_resolution_clock::now();
    autoTimer = std::chrono::high_resolution_clock::now();

    //TODO remove pseudo values
    sensData.lock();
    sensData.sens0 = 100;
    sensData.sens1 = 200;
    sensData.sens2 = 150;
    sensData.sens3 = 125;
    sensData.sens4 = 175;
    sensData.sens5 = 100;
    sensData.unlock();

    motorData.lock();
    motorData.rpm0 = 1870;
    motorData.rpm1 = 4200;
    motorData.rpm2 = 880;
    motorData.rpm3 = 690;
    motorData.unlock();

    batData.lock();
    batData.volt = 12187;
    batData.amps = 4200;
    batData.unlock();
}

Robot::~Robot()
{
    delete btHandler;
    delete uartHandler;
}

void Robot::run()
{
    std::thread btThread(&BTHandler::run, std::ref(btHandler));
    std::thread uartThread(&UARTHandler::run, std::ref(uartHandler));

    while (1)
    {
        thisMode = mode.load();
        thisAutoMode = autoMode.load();
        currentTime = std::chrono::high_resolution_clock::now();

        checkModeChanges();
        checkAutoModeChanges();

        if (thisMode != off && thisMode != idle)
        {
            checkSendUartUpdates();
            checkSendBtUpdates();

            if (thisMode == automatic)
            {
                checkAutoActions();
            }
        }

        lastMode = thisMode;
        lastAutoMode = thisAutoMode;
    }
}

void Robot::checkModeChanges()
{
    if (thisMode != lastMode)
    {
        resetDriveData();

        if (thisMode == off)
        {
            uartHandler->sendStatusData(false, false);
        }
        else if (thisMode == idle)
        {
            uartHandler->sendStatusData(true, false);
        }
        else if (thisMode == manual)
        {
            uartHandler->sendStatusData(true, false);
        }
        else if (thisMode == automatic)
        {
            uartHandler->sendStatusData(true, true);
        }

        if (stateDebug)
        {
            std::cout << "state change: " << lastMode << " -> " << thisMode << "\n";
        }
    }
}

void Robot::checkAutoModeChanges()
{
    if (thisAutoMode != lastAutoMode)
    {
        if (lastAutoMode == learn_back || lastAutoMode == learn_line)
        {
            delete autoLearnData;
        }

        if (stateDebug)
        {
            std::cout << "state change (auto): " << lastAutoMode << " -> " << thisAutoMode << "\n";
        }
    }
}

void Robot::checkSendUartUpdates()
{
    if (currentTime - uartTimer > uartInterval)
    {
        uartHandler->sendDriveData(driveData);

        uartTimer = currentTime;
    }
}

void Robot::checkSendBtUpdates()
{
    if (thisMode == manual)
    {
        if (currentTime - btTimer > btInterval)
        {
            btHandler->sendStatusData(motorData, batData);
            btTimer = currentTime;
        }
    }
    else if (thisMode == automatic)
    {
        if (thisAutoMode == follow || thisAutoMode == route)
        {
            btHandler->sendStatusData(motorData, batData);
            btHandler->sendSensorData(sensData);
            btTimer = currentTime;
        }
    }
}

void Robot::checkAutoActions()
{
    if (thisAutoMode == learn)
    {
        autoMode.store(learn_wait);
        btHandler->sendAutoLearnStatus(learn_wait);
        autoTimer = currentTime;
    }
    else if (thisAutoMode == learn_wait)
    {
        if (currentTime - autoTimer > autoLearnStepTime)
        {
            autoLearnData = new int16_t[autoLearnSteps];
            autoLearnStep = 0;
            autoLowThreshold = 0;
            autoHighThreshold = 0;

            autoMode.store(learn_back);
            btHandler->sendAutoLearnStatus(learn_back);
            autoTimer = currentTime;
        }
    }
    else if (thisAutoMode == learn_back)
    {
        if (currentTime - autoTimer > autoLearnStepTime * autoLearnStep / autoLearnSteps)
        {
            sensData.lock();
            autoLearnData[autoLearnStep++] = (sensData.sens0 + sensData.sens1 + sensData.sens2 + sensData.sens3 + sensData.sens4 + sensData.sens5) / 6;
            sensData.unlock();

            if (autoLearnStep >= autoLearnSteps)
            {
                int learnSum = 0;
                for (int8_t i = 0; i < autoLearnSteps; i++)
                {
                    learnSum += autoLearnData[i];
                }
                autoLowThreshold = learnSum / 10;

                autoMode.store(learn_search);
                btHandler->sendAutoLearnStatus(learn_search);
                autoTimer = currentTime;
            }
        }
    }
    else if (thisAutoMode == learn_search)
    {
        // TODO try to find line here
        if (currentTime - autoTimer > autoLearnStepTime)
        {
            autoLearnData = new int16_t[autoLearnSteps];
            autoLearnStep = 0;

            autoMode.store(learn_line);
            btHandler->sendAutoLearnStatus(learn_line);
            autoTimer = currentTime;
        }
    }
    else if (thisAutoMode == learn_line)
    {
        if (currentTime - autoTimer > autoLearnStepTime * autoLearnStep / autoLearnSteps)
        {
            sensData.lock();
            autoLearnData[autoLearnStep++] = (sensData.sens0 + sensData.sens1 + sensData.sens2 + sensData.sens3 + sensData.sens4 + sensData.sens5) / 6;
            sensData.unlock();

            if (autoLearnStep >= autoLearnSteps)
            {
                int learnSum = 0;
                for (int8_t i = 0; i < autoLearnSteps; i++)
                {
                    learnSum += autoLearnData[i];
                }
                autoHighThreshold = learnSum / 10;

                autoMode.store(learn_end);
                btHandler->sendAutoLearnStatus(learn_end);
                autoTimer = currentTime;

                std::cout << "finished learning with " << autoLowThreshold << " and " << autoHighThreshold << "\n";
            }
        }
    }
}

RobotMode Robot::getRobotMode()
{
    return mode.load();
}

void Robot::setRobotMode(RobotMode _mode)
{
    mode.store(_mode);
}

AutoMode Robot::getAutoMode()
{
    return autoMode.load();
}

void Robot::setAutoMode(AutoMode _mode)
{
    autoMode.store(_mode);
}

DriveData &Robot::getDriveData()
{
    return driveData;
}

void Robot::resetDriveData()
{
    driveData.lock();
    driveData.x = 0.0;
    driveData.y = 0x0;
    driveData.rot = 0;
    driveData.spd = 0;
    driveData.unlock();
}

MotorData &Robot::getMotorData()
{
    return motorData;
}

BatteryData &Robot::getBatteryData()
{
    return batData;
}

SensorData &Robot::getSensorData()
{
    return sensData;
}
