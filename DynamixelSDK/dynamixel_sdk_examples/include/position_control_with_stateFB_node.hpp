//
// 2024.1.22 designed by amby 
//  

#ifndef POS_CTR_W_STATEFB_NODE_HPP_
#define POS_CTR_W_STATEFB_NODE_HPP_

#include <cstdio>
#include <memory>
#include <string>

#include "std_msgs/msg/int32.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"
#include "dynamixel_sdk/dynamixel_sdk.h"
//#include "dynamixel_sdk_custom_interfaces/msg/set_position.hpp"
//#include "dynamixel_sdk_custom_interfaces/srv/get_position.hpp"


class PosCtrNode : public rclcpp::Node
{
public:
  //using SetPosition = dynamixel_sdk_custom_interfaces::msg::SetPosition;
  //using GetPosition = dynamixel_sdk_custom_interfaces::srv::GetPosition;

  PosCtrNode();
  virtual ~PosCtrNode();

private:
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr pos_subscriber;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr led_subscriber;

  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pos_publisher;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr vel_publisher;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr temp_publisher;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr load_publisher;

  rclcpp::TimerBase::SharedPtr timer_;

  void timer_callback();
  
  int cmd_vel;
  int cmd_led;
  
  int sns_pos;
  int sns_vel;
  int sns_temp;
  int16_t sns_load;  
};

#endif  