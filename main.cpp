#include "UART.hpp"
#include "PCB.hpp"
#include <unistd.h>
#include <thread>
//#include "std_msgs/String.h"
//#include "ros/ros.h"

PCB *pcbHandle;

int main(int argc, char const *argv[])
{
   UART uart = UART("/dev/ttyUSB0");
   PCB pcb = PCB(&uart);
   pcbHandle = &pcb;

   /*
   ros::init(argc, argv, "talker");
   ros::NodeHandle ros_handle;
   ros::Publisher publisher = ros_handle.advertise<std_msgs::String>("pcb/ultra", 100);
   ros::Subscriber subscriber = ros_handle.subscribe("pcb/control", 100, controlCallback);
   */
   usleep(100000);
   pcb.setMode(ENABLE);
   usleep(100000);
   pcb.setConfig();
   usleep(100000);
   pcb.setRails(false, false);
   usleep(100000);
   pcb.setMode(SELFCHECK);
   usleep(100000);

   while (true)
   {
      pcb.receiveAndParse();
   }
}

/*
void controlCallback(const std_msgs::String::ConstPtr &msg)
{
   std::string message = msg->data.c_str();
   printf("UART ROS: Received: %s\n", message);

   switch (message)
   {
   case "MODE:ENABLE":
      pcbHandle->setMode(ENABLE);
      break;
   case "MODE:DISABLE":
      pcbHandle->setMode(DISABLE);
      break;

   default:
      printf("UART ROS: ERROR: Unknown ROS Command!\n");
      break;
   }
}
*/