#include "PCB.hpp"

PCB::PCB(UART *uart) : _uart(uart)
{
}

PCB::~PCB()
{
   delete _uart;
}

void PCB::receiveAndParse()
{
   uint8_t data[_uart->getMessageSize()];
   if (_uart->receiveMessage(data))
   {
      parseMessage(data);
   }
}

void PCB::parseMessage(uint8_t *msg)
{
   switch (msg[1])
   {
   case 0x85:
      parseMSG85(msg);
      break;

   default:
      break;
   }
}

void PCB::parseMSG85(uint8_t *msg)
{
   uint16_t vol = (msg[2] << 8) + msg[3];
   int16_t cur = (msg[3] << 8) + msg[4];

   float voltage = vol / 1000.f;
   float current = cur / 1000.f;
}

void PCB::setMode(PCB_MODE mode)
{
   uint8_t msg[_uart->getMessageSize()] = {0x00};

   msg[0] = SET_MODE;
   msg[1] = mode;

   _uart->transmitMessage(msg);
   printf("UART: Set PCB Mode to: %d", mode);
}
