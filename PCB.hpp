#pragma once
#include "UART.hpp"
#include "enums.hpp"
#include "structs.hpp"

enum PCB_MODE
{
   DISABLE = 0x00,
   ENABLE = 0x01,
   START = 0x02,
   IDLE = 0x03,
   SELFCHECK = 0x04,
};

class PCB
{
public:
   PCB(UART *uart);
   ~PCB();

   void setMode(enum PCB_MODE);
   void setConfig();
   void setRails(bool railADJ, bool rail24V);

private:
   UART *_uart;
};