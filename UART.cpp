#include "UART.hpp"

UART::UART(std::string device) : _dev(device)
{

   if (begin())
   {
      printf("UART started!\t\n");
   }
   else
   {
      printf("ERROR: UART start failed!\t\n");
   }
}

UART::~UART()
{
   if (_began)
   {
      close(uart0);
   }
}

bool UART::begin()
{
   if (_began)
   {
      return true;
   }

   //O_RDWR - Open for reading and writing.
   uart0 = open(_dev.c_str(), O_RDWR);
   if (uart0 == -1)
   {
      printf("UART error: unable to open\t\n");
      return false;
   }

   //CLOCAL - Ignore modem status lines
   //CREAD - Enable receiver
   //IGNPAR = Ignore characters with parity errors
   struct termios options;
   tcgetattr(uart0, &options);
   options.c_cflag = B115200 | CS8 | CLOCAL | CREAD;
   options.c_iflag = IGNPAR;
   options.c_oflag = 0;
   options.c_lflag = 0;
   tcflush(uart0, TCIFLUSH);
   tcsetattr(uart0, TCSANOW, &options);

   _began = true;
   return true;
}

void UART::run()
{
   if (uart0 < 0)
   {
      std::cout << "UART error: cannot receive, no uart configured\t\n";
      return;
   }

   while (1)
   {
      int rxLength = read(uart0, (void *)RXBuffer, _messageSize);
      if (rxLength <= 0)
      {
         std::cout << "UART error: no bytes received\n";
         return;
      }

      if (RXBuffer[_messageSize - 1] != _calculateChecksumRX())
      {
         std::cout << "UART error: Wrong Checksum!"
                   << "\t\n";
         return;
      }

      if (RXBuffer[0] != _id)
      {
         std::cout << "UART error: wrong primary id "
                   << "\t\n";
         return;
      }

      switch (RXBuffer[1])
      {

      default:
         std::cout << "UART error: invalid command id "
                   << "\t\n";
         break;
      }
   }
}

void UART::transmitMessage(uint8_t *msg)
{
   if (uart0 < 0)
   {
      printf("ERROR: No UART configured!\n");
      return;
   }

   uint8_t data[_messageSize] = {0xEE};

   if (write(uart0, &data[0], _messageSize) < 0)
   {
      printf("ERROR: Could not send data!\n");
      return;
   }

   printf("UART: Transmit succesfull");
}

void UART::appendChecksum(uint8_t *msg)
{
   msg[_messageSize - 1] = calculateChecksum(msg);
}

uint8_t UART::calculateChecksum(uint8_t *msg)
{
   uint8_t checksum = 0x00;

   for (uint8_t i = 0; i < _messageSize - 1; i++)
   {
      checksum ^= msg[i];
   }

   return checksum;
}

uint8_t UART::_calculateChecksumRX()
{
   return calculateChecksum(RXBuffer);
}