#include "PCB.hpp"

PCB::PCB(UART *uart) : _uart(uart)
{
}

PCB::~PCB()
{
   delete _uart;
}

PUBLISH_MESSAGE PCB::receiveAndParse()
{
   uint8_t data[_uart->getMessageSize()];
   if (_uart->receiveMessage(data))
   {
      return parseMessage(data);
   }

   return UPDATE_NONE;
}

PUBLISH_MESSAGE PCB::parseMessage(uint8_t *msg)
{
   printf("UART: Received Message with ID: %d", msg[1]);

   switch (msg[1])
   {
   case 0x85:
      parseMSG85(msg);
      return UPDATE_BAT;
      break;

   default:
      return UPDATE_NONE;
      break;
   }
}

//Battery Message
void PCB::parseMSG85(uint8_t *msg)
{
   uint16_t vol = (msg[2] << 8) + msg[3];
   int16_t cur = (msg[3] << 8) + msg[4];

   _batVol = vol / 1000.f;
   _batCur = cur / 1000.f;
}

//Temperature message
void PCB::parseMSG86(uint8_t *msg)
{
   uint16_t tempPCB = (msg[2] << 8) + msg[3];
   uint16_t tempIOT = (msg[4] << 8) + msg[5];
   uint16_t tempOUT = (msg[6] << 8) + msg[7];

   _pcbTemp = tempPCB / 100.f;
   _iotTemp = tempIOT / 100.f;
   _outTemp = tempOUT / 100.f;
}

void PCB::setMode(PCB_MODE mode)
{
   uint8_t msg[_uart->getMessageSize()] = {0x00};

   msg[0] = SET_MODE;
   msg[1] = mode;

   _uart->transmitMessage(msg);
   printf("UART: Set PCB Mode to: %d\n", mode);
}
