#include "PCB.hpp"

PCB::PCB(UART *uart) : _uart(uart)
{
}

PCB::~PCB()
{
   delete _uart;
}