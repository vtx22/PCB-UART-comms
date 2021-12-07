#include <iostream>

#include "Robot.hpp"

int main(int argc, char const *argv[])
{
    bool btDebug = false, uartDebug = false, stateDebug = false;

    if (argc > 1)
    {
        for (int i = 1; i < argc; i++)
        {
            if (strcmp(argv[i], "--btdebug") == 0)
            {
                btDebug = true;
            }
            else if (strcmp(argv[i], "--uartdebug") == 0)
            {
                uartDebug = true;
            }
            else if (strcmp(argv[i], "--statedebug") == 0)
            {
                stateDebug = true;
            }
            else
            {
                std::cout << "Error: unknown comannd line argument '" << argv[i] << "'\n";
                return 1;
            }
        }
    }

    Robot robot(btDebug, uartDebug, stateDebug);
    robot.run();

    return 0;
}
