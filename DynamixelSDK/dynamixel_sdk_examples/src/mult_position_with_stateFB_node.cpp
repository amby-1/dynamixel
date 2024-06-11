//
// THis is a smpale program to communicate with multiple dynamixel with high update frequency  
//  Read target angle and P D I gains by messages, and command to dynamixel with set frequency and send messages of the dynamixel states  
// 
//  Sub :: std_msgs::msg::Int32MultiArray cmd/poss  [angle, angle ...] 
//  Sub :: std_msgs::msg::Int32MultiArray cmd/Gains [P I D P I D ...] 
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
//  Realized Speed :: Chk on core i7 14Gen  
//    4Dyna @ 1M Bps (Just read status data) ==> 500Hz MAX
//    4Dyna @ 4M Bps (Just read status data) ==> 1000Hz MAX
//    


/******************************************************************************/
/* include                                                                    */
/******************************************************************************/

#include "dynamixel_sdk/dynamixel_sdk.h"
//#include "dynamixel_sdk_custom_interfaces/msg/set_position.hpp"
//#include "dynamixel_sdk_custom_interfaces/srv/get_position.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"
#include "std_msgs/msg/int32.hpp"

#include "mult_position_with_stateFB_node.hpp"

/******************************************************************************/
/* define                                                                     */
/******************************************************************************/
/* Control table address for X series                                         */
#define ADDR_OPERATING_MODE       11
#define ADDR_LED				          65
#define ADDR_TORQUE_ENABLE				64
#define ADDR_GOAL_CURRENT		  		102	 // Does NOT exist in Rot motors
#define ADDR_GOAL_VELOCITY				104
#define ADDR_GOAL_POSITION				116
#define ADDR_PRESENT_CURRENT			126	 // Represents "Present Load" in Rot motors
#define LEN_PRESENT_CURRENT       2
#define ADDR_PRESENT_VELOCITY			128
#define LEN_PRESENT_VELOCITY      4
#define ADDR_PRESENT_POSITION			132
#define LEN_PRESENT_POSITION      4
#define ADDR_PRESENT_TEMPERATURE	146
#define LEN_PRESENT_TEMPERATURE   1
#define ADDR_PRESENT_VOLTAGE  	  144
#define LEN_PRESENT_VOLTAGE       2

#define ADDR_GAINS				        80
#define LEN_GAINS                 6
 //80	2	Position D Gain	RW	0	0 ~ 16,383	-
 //82	2	Position I Gain	RW	0	0 ~ 16,383	-
 //84	2	Position P Gain	

#define ADDR_STATUS               126
#define LEN_STATUS                21
// 126	2	Present Load	R	-	-1,000 ~ 1,000	0.1 [%]
// 128	4	Present Velocity	R	-	-	0.229 [rev/min]
// 132	4	Present Position	R	-	-	1 [pulse]
// 136	4	Velocity Trajectory	R	-	-	0.229 [rev/min]
// 140	4	Position Trajectory	R	-	-	1 [pulse]
// 144	2	Present Input Voltage	R	-	-	0.1 [V]
// 146	1	Present Temperature	R	-	-	1 [°C]

#define LEN_GOAL_POSITION         4
#define LEN_LED                   1  

/* TORQUE ENABLE/DISABLE */
#define TORQUE_ENABLE                   1	 // Value for enabling the torque
#define TORQUE_DISABLE                  0	 // Value for disabling the torque

/* OPERATING_MODE */
#define CURRENT_BASED_POSITION_CONTROL 5
#define POSITION_CONTROL				       3
#define VELOCITY_CONTROL				       1
#define TORQUE_CONTROL					       0

/*COMMAND TYPE*/
#define SEND_POSITION 0
#define SEND_LED  1
#define SEND_GAINS 2

#define ERROR_VALUE -256*256*256*128 + 1

/* Protocol version */ 
#define PROTOCOL_VERSION 2.0  // Default Protocol version of DYNAMIXEL X series.

/* Default setting */
#define BAUDRATE  4000000 //57600  // Default Baudrate of DYNAMIXEL X series
#define DEVICE_NAME "/dev/ttyUSB0"  // [Linux]: "/dev/ttyUSB*", [Windows]: "COM*"

#define LOOP_DURATION 2ms // it determines the control frequency of Dynamixel
// Dynamixel ID
//  Specify the Dynamixel IDs that is connected and controlled
//static std::array<int, 4> DYNA_IDs = {0x01, 0x02, 0x03, 0x04};
static std::array<int, 4> DYNA_IDs = {0x01, 0x02, 0x03, 0x04};


using namespace std::chrono_literals;

dynamixel::PortHandler * portHandler;
dynamixel::PacketHandler * packetHandler;

uint8_t dxl_error = 0;
uint32_t goal_position = 0;
int dxl_comm_result = COMM_TX_FAIL;

/******************************************************************************/
/* Constructor                                                                */
/******************************************************************************/
MultPosCtrNode::MultPosCtrNode()
: Node("mult_position_control_node"), 
  groupSyncWritePosition(portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION), 
  groupSyncWriteLED(portHandler, packetHandler, ADDR_LED, LEN_LED), 
  groupSyncWriteGain(portHandler, packetHandler, ADDR_GAINS, LEN_GAINS), 
  groupSyncRead(portHandler, packetHandler, ADDR_STATUS, LEN_STATUS)
{
  RCLCPP_INFO(this->get_logger(), "Run read write node");
  N = DYNA_IDs.size(); 

  for (int i=0; i<3; i++){
    send_flag.at(i) = 0;
  }
  cmd_poss.resize(N);
  cmd_leds.resize(N);
  cmd_P_gains.resize(N);
  cmd_I_gains.resize(N);
  cmd_D_gains.resize(N);

  this->declare_parameter("qos_depth", 10);
  int8_t qos_depth = 0;
  this->get_parameter("qos_depth", qos_depth);

  const auto QOS_RKL10V =
    rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();

  pos_subscriber = 
    this->create_subscription<std_msgs::msg::Int32MultiArray>(
    "cmd/poss",
    QOS_RKL10V,
    [this](const std_msgs::msg::Int32MultiArray::SharedPtr msg) -> void
    {
      if(msg->data.size() == N){
        for(unsigned int i=0; i<N; i++){
          cmd_poss.at(i) = msg->data.at(i);
        }
        send_flag.at(SEND_POSITION) = 1;
      }else{
        RCLCPP_WARN(rclcpp::get_logger("mult_position_control_node"), "Size of commanded position array is not correct");
      }
    }
    );

  led_subscriber = 
    this->create_subscription<std_msgs::msg::Int32MultiArray>(
    "cmd/leds",
    QOS_RKL10V,
    [this](const std_msgs::msg::Int32MultiArray::SharedPtr msg) -> void
    {
      if(msg->data.size() == N){
        for(unsigned int i=0; i<N; i++){
          cmd_leds.at(i) = msg->data.at(i);
        }
        send_flag.at(SEND_LED) = 1;
      }else{
        RCLCPP_WARN(rclcpp::get_logger("mult_position_control_node"), "Size of commanded LED array is not correct");
      }
    }
    );


  gain_subscriber = 
    this->create_subscription<std_msgs::msg::Int32MultiArray>(
    "cmd/gains",
    QOS_RKL10V,
    [this](const std_msgs::msg::Int32MultiArray::SharedPtr msg) -> void
    {
      if(msg->data.size() == 3*N){
        for(unsigned int i=0; i<N; i++){
          cmd_P_gains.at(i) = msg->data.at(3*i);
          cmd_I_gains.at(i) = msg->data.at(3*i+1);
          cmd_D_gains.at(i) = msg->data.at(3*i+2);
        }
        send_flag.at(SEND_GAINS) = 1;
      }else{
        RCLCPP_WARN(rclcpp::get_logger("mult_position_control_node"), "Size of commanded Gain array is not correct");
      }
    }
    );
  
  pos_publisher = this->create_publisher<std_msgs::msg::Int32MultiArray>("sns/poss", 10);
  vel_publisher = this->create_publisher<std_msgs::msg::Int32MultiArray>("sns/vels", 10);
  temp_publisher = this->create_publisher<std_msgs::msg::Int32MultiArray>("sns/temps", 10);
  load_publisher = this->create_publisher<std_msgs::msg::Int32MultiArray>("sns/loads", 10);
  volt_publisher = this->create_publisher<std_msgs::msg::Int32MultiArray>("sns/volts", 10);
  
  timer_ = this->create_wall_timer(LOOP_DURATION, std::bind(&MultPosCtrNode::timer_callback, this));

  // Initialize dynamixel sync read class 
  for(unsigned int i=0; i<N; i++){
    if(groupSyncRead.addParam(DYNA_IDs.at(i)) != true){
      RCLCPP_ERROR(rclcpp::get_logger("mult_position_control_node"), "Failed to add Param to groupSyncRead class");
    }
  }

  //get_position_server_ = create_service<GetPosition>("get_position", get_present_position);
}

MultPosCtrNode::~MultPosCtrNode()
{
}

void MultPosCtrNode::timer_callback(){
    static int count = 0;
    count++;
    // Timer callback function ----
    // 
    // First READ STATUS, Then send COMMANDS 
    auto pos_message = std_msgs::msg::Int32MultiArray();
    auto vel_message = std_msgs::msg::Int32MultiArray();
    auto temp_message = std_msgs::msg::Int32MultiArray();
    auto volt_message = std_msgs::msg::Int32MultiArray();
    auto load_message = std_msgs::msg::Int32MultiArray();
    pos_message.data.resize(N);
    vel_message.data.resize(N);
    temp_message.data.resize(N);
    volt_message.data.resize(N);
    load_message.data.resize(N);   

    // READ ALL DATA Packet with one command
    dxl_comm_result = groupSyncRead.txRxPacket();
    if (dxl_comm_result != COMM_SUCCESS){
        //packetHandler->printTxRxResult(dxl_comm_result);
        RCLCPP_ERROR(rclcpp::get_logger("mult_position_control_node"), "Failed to add Param to groupSyncRead class");
    }

    // get STATUS Data from the received packet for all IDs 
    for (unsigned int i=0; i<N; i++){
      // Check if groupsyncread data of Dynamixel#1 is available
      bool dxl_getdata_result = groupSyncRead.isAvailable(DYNA_IDs.at(i), ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
      if (dxl_getdata_result != true){
        RCLCPP_ERROR(rclcpp::get_logger("mult_position_control_node"), "Failed to read [ID:%03d]", DYNA_IDs.at(i));
        pos_message.data.at(i) = ERROR_VALUE; 
        vel_message.data.at(i) = ERROR_VALUE;
        temp_message.data.at(i) = ERROR_VALUE;
        volt_message.data.at(i) = ERROR_VALUE;
        load_message.data.at(i) = ERROR_VALUE;
      }else{
        // get Data of this ID
        pos_message.data.at(i) = groupSyncRead.getData(DYNA_IDs.at(i), ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
        vel_message.data.at(i) = groupSyncRead.getData(DYNA_IDs.at(i), ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY);
        temp_message.data.at(i) = groupSyncRead.getData(DYNA_IDs.at(i), ADDR_PRESENT_TEMPERATURE, LEN_PRESENT_TEMPERATURE);
        volt_message.data.at(i) = groupSyncRead.getData(DYNA_IDs.at(i), ADDR_PRESENT_VOLTAGE, LEN_PRESENT_VOLTAGE);
        load_message.data.at(i) = (int16_t)groupSyncRead.getData(DYNA_IDs.at(i), ADDR_PRESENT_CURRENT, LEN_PRESENT_CURRENT);
      }
    }
      
    // Inform situation 
    if(count %100 == 1){
      for (unsigned int i=0; i<N; i++){
        RCLCPP_INFO(
          this->get_logger(),
          "Get [IDs: %d] [Position: %d] [Velocity: %d] [LOAD: %d] [TEMP: %d]",
          DYNA_IDs.at(i), pos_message.data.at(i), vel_message.data.at(i), load_message.data.at(i), temp_message.data.at(i)
        );
      }
    }
    
    // SEND Packet depending on the commands
    //  in the case of position send command 
    if( send_flag.at(SEND_POSITION) ){
      // prepare for packet data
      groupSyncWritePosition.clearParam();
      for (unsigned int i=0; i<N; i++){
        uint8_t gPos[4];
        // Allocate goal position value into byte array
        gPos[0] = DXL_LOBYTE(DXL_LOWORD(cmd_poss.at(i)));
        gPos[1] = DXL_HIBYTE(DXL_LOWORD(cmd_poss.at(i)));
        gPos[2] = DXL_LOBYTE(DXL_HIWORD(cmd_poss.at(i)));
        gPos[3] = DXL_HIBYTE(DXL_HIWORD(cmd_poss.at(i)));

        if( groupSyncWritePosition.addParam(DYNA_IDs.at(i), gPos) != true){
          RCLCPP_ERROR(rclcpp::get_logger("mult_position_control_node"), "Failed to set position of [ID:%03d]", DYNA_IDs.at(i));
        }
      }
      // send packet 
      dxl_comm_result = groupSyncWritePosition.txPacket();
      if (dxl_comm_result != COMM_SUCCESS){
        //packetHandler->printTxRxResult(dxl_comm_result);
        RCLCPP_ERROR(rclcpp::get_logger("mult_position_control_node"), "Failed to send packet of position command");
      }
      // clear flag and finish 
      send_flag.at(SEND_POSITION) = 0;
    }

    // in the case of LED send command
    if( send_flag.at(SEND_LED) ){
      // prepare for packet data
      groupSyncWriteLED.clearParam();
      for (unsigned int i=0; i<N; i++){
        uint8_t led;
        // Allocate goal position value into byte array
        led = static_cast<u_int8_t>(cmd_leds.at(i));
        if( groupSyncWriteLED.addParam(DYNA_IDs.at(i), &led) != true){
          RCLCPP_ERROR(rclcpp::get_logger("mult_position_control_node"), "Failed to set LED of [ID:%03d]", DYNA_IDs.at(i));
        }
      }
      // send packet 
      dxl_comm_result = groupSyncWriteLED.txPacket();
      if (dxl_comm_result != COMM_SUCCESS){
        //packetHandler->printTxRxResult(dxl_comm_result);
        RCLCPP_ERROR(rclcpp::get_logger("mult_position_control_node"), "Failed to send packet of LED command");
      }
      // clear flag and finish 
      send_flag.at(SEND_LED) = 0;
    }

    // in the case of Gain send command
    if( send_flag.at(SEND_GAINS) ){
      // prepare for packet data
      groupSyncWriteGain.clearParam();
      for (unsigned int i=0; i<N; i++){
        uint8_t gain[6];
        // Allocate goal position value into byte array
        gain[0] = DXL_LOBYTE(cmd_D_gains.at(i));
        gain[1] = DXL_HIBYTE(cmd_D_gains.at(i));
        gain[2] = DXL_LOBYTE(cmd_I_gains.at(i));
        gain[3] = DXL_HIBYTE(cmd_I_gains.at(i));
        gain[4] = DXL_LOBYTE(cmd_P_gains.at(i));
        gain[5] = DXL_HIBYTE(cmd_P_gains.at(i));
        if( groupSyncWriteGain.addParam(DYNA_IDs.at(i), gain) != true){
          RCLCPP_ERROR(rclcpp::get_logger("mult_position_control_node"), "Failed to set Gain of [ID:%03d]", DYNA_IDs.at(i));
        }
      }
      // send packet 
      dxl_comm_result = groupSyncWriteGain.txPacket();
      if (dxl_comm_result != COMM_SUCCESS){
        //packetHandler->printTxRxResult(dxl_comm_result);
        RCLCPP_ERROR(rclcpp::get_logger("mult_position_control_node"), "Failed to send packet of Gain command");
      }
      // clear flag and finish 
      send_flag.at(SEND_GAINS) = 0;
    }

    // publish data 
    pos_publisher->publish(pos_message);
    vel_publisher->publish(vel_message);
    temp_publisher->publish(temp_message);
    load_publisher->publish(load_message);
    volt_publisher->publish(volt_message);
};


/******************************************************************************/
/* Function                                                                   */
/******************************************************************************/
void setupDynamixel(uint8_t dxl_id)
{
  /* Use Position Control Mode                */
  /* #define CURRENT_BASED_POSITION_CONTROL 5 */
  /* #define POSITION_CONTROL				        3 */
  /* #define VELOCITY_CONTROL				        1 */
  /* #define TORQUE_CONTROL					        0 */
  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler,
    dxl_id,
    ADDR_OPERATING_MODE,
    POSITION_CONTROL,
    &dxl_error
  );

  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("mult_position_control_node"), "Failed to set Position Control Mode.");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("mult_position_control_node"), "Succeeded to set Position Control Mode.");
  }

  // Enable Torque of DYNAMIXEL
  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler,
    dxl_id,
    ADDR_TORQUE_ENABLE,
    TORQUE_ENABLE,  /* Torque ON */
    &dxl_error
  );

  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("mult_position_control_node"), "Failed to enable torque.");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("mult_position_control_node"), "Succeeded to enable torque.");
  }
}

int main(int argc, char * argv[])
{
  portHandler = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
  packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  // Open Serial Port
  dxl_comm_result = portHandler->openPort();
  if (dxl_comm_result == false) {
    RCLCPP_ERROR(rclcpp::get_logger("position_control_node"), "Failed to open the port!");
    return -1;
  } else {
    RCLCPP_INFO(rclcpp::get_logger("position_control_node"), "Succeeded to open the port.");
  }

  // Set the baudrate of the serial port (use DYNAMIXEL Baudrate)
  dxl_comm_result = portHandler->setBaudRate(BAUDRATE);
  if (dxl_comm_result == false) {
    RCLCPP_ERROR(rclcpp::get_logger("position_control_node"), "Failed to set the baudrate!");
    return -1;
  } else {
    RCLCPP_INFO(rclcpp::get_logger("position_control_node"), "Succeeded to set the baudrate.");
  }

  setupDynamixel(BROADCAST_ID);

  rclcpp::init(argc, argv);

  auto posCtrNode = std::make_shared<MultPosCtrNode>();
  rclcpp::spin(posCtrNode);

  // Disable Torque of DYNAMIXEL
  packetHandler->write1ByteTxRx(
    portHandler,
    BROADCAST_ID,
    ADDR_TORQUE_ENABLE,
    TORQUE_DISABLE,
    &dxl_error
  );
  portHandler->closePort();
	
	rclcpp::shutdown();

  return 0;
}
