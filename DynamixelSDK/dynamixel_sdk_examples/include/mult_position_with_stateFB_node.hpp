//
// 2024.6.10 designed by amby 
//  
// THis is a smpale program to communicate with multiple dynamixel with high update frequency  
//  Read target angle and P D I gains by messages, and command to dynamixel with set frequency and send messages of the dynamixel states  
// 
//  Sub :: std_msgs::msg::Int32MultiArray cmd/poss  [angle, angle ...] 
//  Sub :: std_msgs::msg::Int32MultiArray cmd/Gains [P I D, P I D, ...] 
//
//  Pub :: std_msgs::msg::Int32MultiArray sns/poss  [angle, angle ...]
//  Pub :: std_msgs::msg::Int32MultiArray sns/vels  [vel, vel ...]
//  Pub :: std_msgs::msg::Int32MultiArray sns/loads  [load, load ...]
//  Pub :: std_msgs::msg::Int32MultiArray sns/temps  [tmp, tmp ...]
//  Pub :: std_msgs::msg::Int32MultiArray sns/volts  [v, v ...]
//
//  Those values are digit data, please transform them depending on the dynamixel specification. 
//  see e-mannual of Dynamixel
//  
//  This program relies on Dynamixel SDK


#ifndef MULT_POS_CTR_W_STATEFB_NODE_HPP_
#define MULT_POS_CTR_W_STATEFB_NODE_HPP_

#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"
#include "dynamixel_sdk/dynamixel_sdk.h"
//#include "dynamixel_sdk_custom_interfaces/msg/set_position.hpp"
//#include "dynamixel_sdk_custom_interfaces/srv/get_position.hpp"


class MultPosCtrNode : public rclcpp::Node
{
public:
  //using SetPosition = dynamixel_sdk_custom_interfaces::msg::SetPosition;
  //using GetPosition = dynamixel_sdk_custom_interfaces::srv::GetPosition;

  MultPosCtrNode();
  virtual ~MultPosCtrNode();

private:
  //  subscriber 
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr pos_subscriber;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr led_subscriber;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr gain_subscriber;
  // pbulisher
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pos_publisher;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr vel_publisher;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr temp_publisher;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr load_publisher;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr volt_publisher;

  // timer
  rclcpp::TimerBase::SharedPtr timer_;
  void timer_callback();

  // Dynamixel Class to send the same command to multiple Dynamixel at the same time
  dynamixel::GroupSyncWrite groupSyncWritePosition; // Class to send position at the same time 
  dynamixel::GroupSyncWrite groupSyncWriteLED; // Class to send led at the same time 
  dynamixel::GroupSyncWrite groupSyncWriteGain; // Class to send gain at the same time 

  // Dynamixel Class to read the datra from multiple Dynamixel at the same time
  dynamixel::GroupSyncRead groupSyncRead; // Read the status data at the same time

  // variables of sensing and commanding   
  unsigned int N; // number of dynamixel
  std::array<int, 3> send_flag; // whether new command reaches or not  for 0:Pos, 1:LED, 2:Gain commands 
  std::vector<int> cmd_poss;
  std::vector<int> cmd_P_gains;
  std::vector<int> cmd_D_gains;
  std::vector<int> cmd_I_gains;
  std::vector<int> cmd_leds;
  
  /*
  std::vector<int> sns_poss;
  std::vector<int> sns_vels;
  std::vector<int> sns_temps;
  std::vector<int> sns_loads;
  std::vector<int> sns_volts;
  int16_t sns_load;  
  */
};

#endif  