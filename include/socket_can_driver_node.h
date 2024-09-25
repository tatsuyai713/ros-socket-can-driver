#ifndef _SOCKET_CAN_DRIVER_H_
#define _SOCKET_CAN_DRIVER_H_
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include "rclcpp/rclcpp.hpp"

#include <linux/can.h>
#include <linux/can/raw.h>

#include <vector>
#include <string>
#include <thread>
#include <mutex>

#include <boost/thread.hpp>

#include "yaml-cpp/yaml.h"
#include <fstream>

#include "ros_socket_can_driver/msg/can_frame.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>



enum class socket_can_exception
{
  OPEN_FAIL,
  CONFIG_FAIL,
  BIND_FAIL,
  RECV_ERR,
  NO_MSG,
  TIMEOUT,
};

typedef struct CANMsg
{
  uint8_t device_channel;
  uint16_t can_id;
  uint8_t bypass_channel;
} CANMsg;

typedef struct CANConfig
{
  std::vector<CANMsg> msgs;
} CANConfig;

class SocketCanDriver : public rclcpp::Node {
public:
    SocketCanDriver();
    ~SocketCanDriver();
private:
  int device_channel_num_;
  int can_bitrate_;
  bool virtual_;

  CANConfig can_tx_config_;
  CANConfig can_rx_config_;
  std::string can_tx_config_path_;
  std::string can_rx_config_path_;

	std::vector<std::thread> receive_thread_list_;
	std::vector<std::thread> send_thread_list_;
	std::vector<int> socket_hnd_;
  std::vector<struct sockaddr_can> addr_list_;

	std::mutex *socket_mtx_;

	// callback functions
	void cantxCallback(const ros_socket_can_driver::msg::CANFrame::SharedPtr msg);

	// thread
	void canSendThread(uint8_t channel_num);
	void canReceiveThread(uint8_t channel_num);

  // Subscriber
  rclcpp::Subscription<ros_socket_can_driver::msg::CANFrame>::SharedPtr socket_can_tx_sub_;
  rclcpp::Publisher<ros_socket_can_driver::msg::CANFrame>::SharedPtr socket_can_rx_pub_; 

};

#endif // _SOCKET_CAN_DRIVER_H_