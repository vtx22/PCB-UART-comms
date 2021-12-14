#include "UART.hpp"

UART::UART(std::string device) : _dev(device)
{

   if (begin())
   {
      printf("UART: Started UART\n");
   }
   else
   {
      printf("UART ERROR: UART start failed!\n");
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
      printf("UART ERROR: UART already open!\n");
      return true;
   }

   //O_RDWR - Open for reading and writing.
   uart0 = open(_dev.c_str(), O_RDWR);
   if (uart0 == -1)
   {
      printf("UART ERROR: Unable to open port!\n");
      return false;
   }

   //CLOCAL - Ignore modem status lines
   //CREAD - Enable receiver
   //IGNPAR = Ignore characters with parity errors
   struct termios options;
   tcgetattr(uart0, &options);
   options.c_cflag |= B115200 | CS8 | CLOCAL | CREAD;
   options.c_cflag &= ~PARENB;
   options.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
   options.c_oflag |= 0;
   options.c_lflag &= ~ICANON;
   options.c_cc[VTIME] = 0;
   options.c_cc[VMIN] = _messageSizeRX;
   tcflush(uart0, TCIFLUSH);
   tcsetattr(uart0, TCSANOW, &options);

   _began = true;
   return true;
}
/*
void UART::run()
{
   if (uart0 < 0)
   {
      std::cout << "UART ERROR: No UART configured!\n";
      return;
   }

   while (1)
   {
      int rxLength = read(uart0, (void *)RXBuffer, _messageSize);
      if (rxLength <= 0)
      {
         std::cout << "UART ERROR: no bytes received\n";
         return;
      }

      if (RXBuffer[_messageSize - 1] != _calculateChecksumRX())
      {
         std::cout << "UART ERROR: Wrong Checksum!"
                   << "\n";
         return;
      }

      if (RXBuffer[0] != _id)
      {
         std::cout << "UART ERROR: wrong primary id "
                   << "\n";
         return;
      }

      switch (RXBuffer[1])
      {

      default:
         std::cout << "UART ERROR: invalid command id "
                   << "\n";
         break;
      }
   }
}
*/
void UART::transmitMessage(uint8_t *msg)
{
   if (uart0 < 0)
   {
      printf("UART ERROR: No UART configured!\n");
      return;
   }

   uint8_t message[_messageSize];

   message[0] = _id;
   for (uint8_t cnt = 1; cnt < _messageSize - 2; cnt++)
   {
      message[cnt] = msg[cnt - 1];
   }
   appendChecksum(message);

   if (write(uart0, message, _messageSize) < 0)
   {
      printf("UART ERROR: Could not send data!\n");
      return;
   }

   printf("UART: Transmit successfull\n");
}

bool UART::receiveMessage(uint8_t *msg)
{
   if (read(uart0, (void *)RXBuffer, _messageSizeRX) <= 0)
   {
      printf("UART ERROR: No message received!\n");
      return false;
   }

   if (!checkMessage())
   {
      printf("UART ERROR: Message Check failed!\n");
      return false;
   }

   for (uint8_t cnt = 0; cnt < _messageSizeRX; cnt++)
   {
      msg[cnt] = RXBuffer[cnt];
   }

   return true;
}

bool UART::checkMessage()
{
   if (_calculateChecksumRX() != RXBuffer[_messageSize - 1])
   {
      printf("UART ERROR: Wrong Checksum!\n");
      return false;
   }
   if (RXBuffer[0] != _id)
   {
      printf("UART ERROR: IDs do not match!\n");
      return false;
   }

   return true;
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