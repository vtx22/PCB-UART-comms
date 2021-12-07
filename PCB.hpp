#pragma once
#include "UART.hpp"

enum PCB_MODE
{
   DISABLE = 0x00,
   ENABLE = 0x01,
   START = 0x02,
   IDLE = 0x03,
   SELFCHECK = 0x04,
} typedef PCB_MODE;

const uint8_t SET_MODE = 0x05;
const uint8_t SET_RAIL = 0x06;

class PCB
{
public:
   PCB(UART *uart);
   ~PCB();

   void setMode(PCB_MODE mode);
   void setConfig();
   void setRails(bool railADJ, bool rail24V);

private:
   UART *_uart;
};