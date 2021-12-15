#include "UART.hpp"
#include "PCB.hpp"
#include <unistd.h>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

PCB *pcbHandle;

int main(int argc, char const *argv[])
{
   UART uart = UART("/dev/ttyUSB0");
   PCB pcb = PCB(&uart);
   pcbHandle = &pcb;
   /*
   rclcpp::init(argc, argv, "pcb_talker");
   ros::NodeHandle ros_handle;
   ros::Publisher pubULTRA = ros_handle.advertise<std_msgs::String>("pcb/ultra", 100);
   ros::Publisher pubBAT = ros_handle.advertise<std_msgs::String>("pcb/battery", 100);
   ros::Publisher pubTEMP = ros_handle.advertise<std_msgs::String>("pcb/temperatures", 100);
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

   MinimalPublisher* batPub = std::make_shared<MinimalPublisher>();

   while (true)
   {
      /*
      PUBLISH_MESSAGE updateMessage = pcb.receiveAndParse();

      if (updateMessage == UPDATE_NONE)
      {
         continue;
      }

      std_msgs::String msg;
      std::stringstream ss;

      switch (updateMessage)
      {
      default:
      case UPDATE_NONE:
         break;
      case UPDATE_BAT:
         ss << "VOLTAGE:" << pcb.getBatVol();
         msg.data = ss.str();
         pubBAT.publish(msg);
         ss << "CURRENT:" << pcb.getBatCur();
         msg.data = ss.str();
         pubBAT.publish(msg);
         printf("UART ROS: Published Battery status\n");
         break;
      case UPDATE_TEMP:
         ss << "PCBTEMP:" << pcb.getPCBTemp();
         msg.data = ss.str();
         pubTEMP.publish(msg);
         ss << "IOTTEMP:" << pcb.getIOTTemp();
         msg.data = ss.str();
         pubTEMP.publish(msg);
         printf("UART ROS: Published Temperature status\n");
         break;
      }
      */

     batPub->publishMessage("TEST!");
      rclcpp::spin_once();   
      usleep(100000);
   }
}

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


class MinimalPublisher : public rclcpp::Node
{
  public:
   MinimalPublisher() : Node("minimal_publisher"), count_(0)
   {
      publisher_ = this->create_publisher<std_msgs::msg::String>("pcb/battery", 10);
   }
   
   void publishMessage(std::string msg) 
   {
      auto message = std_msgs::msg::String();
      message.data = msg;
      publisher_->publish(message);
      printf("UART ROS: Published message!");
   }

   
  private:
   rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
   rclcpp::TimerBase::SharedPtr timer_;
    
   size_t count_;
};
