#pragma once

#include <stdint.h>
#include <fcntl.h>
#include <iostream>
#include <sstream>
#include <termios.h>
#include <unistd.h>
#include <string.h>

class UART
{
public:
   UART(std::string device);
   ~UART();

   bool begin();
   uint8_t getMessageSize() { return _messageSize; }
   void transmitMessage(uint8_t *msg);
   bool receiveMessage(uint8_t *msg);

private:
   void appendChecksum(uint8_t *msg);
   bool checkMessage();

   uint8_t calculateChecksum(uint8_t *msg);
   uint8_t _calculateChecksumRX();
   int uart0 = {-1};

   bool _began = false;

   const uint8_t _id = 0x69;
   std::string _dev;

   const static uint8_t _messageSize = 12; //Message size in bytes
   const static uint8_t _messageSizeRX = 12;
   uint8_t TXBuffer[_messageSize];
   uint8_t RXBuffer[_messageSizeRX];
};