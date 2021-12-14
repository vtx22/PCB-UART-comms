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

enum PUBLISH_MESSAGE
{
   UPDATE_NONE = 0x00,
   UPDATE_BAT = 0x01,
   UPDATE_TEMP = 0x02,

} typedef PUBLISH_MESSAGE;

const uint8_t SET_MODE = 0x05;
const uint8_t SET_RAIL = 0x06;

class PCB
{
public:
   PCB(UART *uart);
   ~PCB();

   PUBLISH_MESSAGE receiveAndParse();

   void setMode(PCB_MODE mode);
   void setConfig(){};
   void setRails(bool railADJ, bool rail24V){};

   const float getBatVol() { return _batVol; };
   const float getBatCur() { return _batCur; };
   const float getPCBTemp() { return _pcbTemp; };
   const float getIOTTemp() { return _iotTemp; };

private:
   float _batVol = 0.f;
   float _batCur = 0.f;
   float _pcbTemp = 0.f;
   float _iotTemp = 0.f;
   float _outTemp = 0.f;

   PUBLISH_MESSAGE parseMessage(uint8_t *msg);
   void parseMSG85(uint8_t *msg);
   oid parseMSG86(uint8_t *msg);
   UART *_uart;
};