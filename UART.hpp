#pragma once

#include <stdint.h>
#include <fcntl.h>
#include <iostream>
#include <sstream>
#include <termios.h>

class UART
{
public:
   UART(std::string device);
   ~UART();

   bool begin();
   void run();

   void transmitMessage(uint8_t *msg);

private:
   void appendChecksum(uint8_t *msg);

   uint8_t calculateChecksum(uint8_t *msg);
   uint8_t _calculateChecksumRX();
   std::atomic<int> uart0{-1};

   bool _began = false;

   const uint8_t _id = 0x69;
   std::string _dev;

   const static uint8_t _messageSize = 12; //Message size in bytes
   uint8_t RXBuffer[_messageSize];
};