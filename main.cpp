#include "UART.hpp"
#include "PCB.hpp"
#include "ros/ros.h"

int main(int argc, char const *argv[])
{
   UART uart = UART("/dev/ttyUSB0");
   PCB pcb = PCB(&uart);

   pcb.setMode(ENABLE);
}