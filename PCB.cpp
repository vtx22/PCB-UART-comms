#include "PCB.hpp"

PCB::PCB(UART *uart) : _uart(uart)
{
}

PCB::~PCB()
{
   delete _uart;
}

void PCB::setMode(PCB_MODE mode)
{
   uint8_t msg[_uart->getMessageSize()] = {0x00};

   msg[0] = SET_MODE;
   msg[1] = mode;

   _uart->transmitMessage(msg);
}