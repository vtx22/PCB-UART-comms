#include "UART.hpp"
#include "PCB.hpp"

int main(int argc, char const *argv[])
{
   UART uart = UART("dev/ttyUSB0");
   PCB pcb = PCB(&uart);

   while (1)
   {
   }
}