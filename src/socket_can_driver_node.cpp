#include "socket_can_driver_node.h"

#define BUFF_SIZE 256


SocketCanDriver::SocketCanDriver() : Node("socket_can_driver_node")
{

  // パラメータの取得
  this->declare_parameter("device_channel_num", 1);
  this->get_parameter("device_channel_num", device_channel_num_);
  RCLCPP_INFO(this->get_logger(), "device_channel_num : %d", device_channel_num_ );
  int bitrate = 0;
  this->declare_parameter("can_bitrate", 500000);
  this->get_parameter("can_bitrate", bitrate);
  RCLCPP_INFO(this->get_logger(), "bitrate : %d", bitrate );

  this->declare_parameter("can_tx_config_path", "can_tx_config.yaml");
  this->get_parameter("can_tx_config_path", can_tx_config_path_);
  this->declare_parameter("can_rx_config_path", "can_rx_config.yaml");
  this->get_parameter("can_rx_config_path", can_rx_config_path_);
  std::string package_path = ament_index_cpp::get_package_share_directory("ros_kvaser_can_driver");
  can_tx_config_path_ = package_path + "/config/" + can_tx_config_path_;
  can_rx_config_path_ = package_path + "/config/" + can_rx_config_path_;

  // ファイルからYAMLデータを読み込む
  std::ifstream can_tx_config_fin(can_tx_config_path_); // ファイルのパスを指定
  std::ifstream can_rx_config_fin(can_rx_config_path_); // ファイルのパスを指定
  // ファイルが開けたかを確認
  if (!can_tx_config_fin)
  {
    RCLCPP_ERROR(this->get_logger(), "File not found or unable to open" );
    rclcpp::shutdown();
    return;
  }
  if (!can_rx_config_fin)
  {
    RCLCPP_ERROR(this->get_logger(), "File not found or unable to open" );
    rclcpp::shutdown();
    return;
  }

  std::stringstream can_tx_config_buffer;
  can_tx_config_buffer << can_tx_config_fin.rdbuf();
  std::string can_tx_config_yaml_data = can_tx_config_buffer.str();
  YAML::Node can_tx_config_yaml = YAML::Load(can_tx_config_yaml_data);
  for (auto const &msg : can_tx_config_yaml["can_tx_config"])
  {
    CANMsg can_msg;
    can_msg.device_channel = msg["device_channel"].as<uint8_t>();
    can_msg.can_id = msg["can_id"].as<uint16_t>();
    can_msg.bypass_channel = -1;
    can_tx_config_.msgs.push_back(can_msg);
  }

  std::stringstream can_rx_config_buffer;
  can_rx_config_buffer << can_rx_config_fin.rdbuf();
  std::string can_rx_config_yaml_data = can_rx_config_buffer.str();
  YAML::Node can_rx_config_yaml = YAML::Load(can_rx_config_yaml_data);
  for (auto const &msg : can_rx_config_yaml["can_rx_config"])
  {
    CANMsg can_msg;
    can_msg.device_channel = msg["device_channel"].as<uint8_t>();
    can_msg.can_id = msg["can_id"].as<uint16_t>();
    can_msg.bypass_channel = msg["bypass_channel"].as<uint8_t>();
    can_rx_config_.msgs.push_back(can_msg);
  }

  // Existing subscription logic
  socket_can_tx_sub_ = this->create_subscription<ros_socket_can_driver::msg::CANFrame>(
      "can_tx", 10, std::bind(&SocketCanDriver::cantxCallback, this, std::placeholders::_1));

  // Publisher creation logic for custom message
  socket_can_rx_pub_ = this->create_publisher<ros_socket_can_driver::msg::CANFrame>("can_rx", 10);

  receive_thread_list_.clear();
  socket_mtx_ = new std::mutex[device_channel_num_];

  // socket can initialize
  for (int i = 0; i < device_channel_num_; ++i)
  {
    try
    {
      // Open can socket
      int s;
      if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
      {
        RCLCPP_ERROR(this->get_logger(), "socket open error");
        throw(socket_can_exception::OPEN_FAIL);
      }
      RCLCPP_INFO(this->get_logger(), "Open %d channel", i);

      int rcvbuf_size = BUFF_SIZE;
      if (setsockopt(s, SOL_SOCKET, SO_RCVBUF,
                     &rcvbuf_size, sizeof(rcvbuf_size)) < 0)
      {
        RCLCPP_ERROR(this->get_logger(), "setsockopt SO_RCVBUF");
        throw(socket_can_exception::CONFIG_FAIL);
      }

      struct sockaddr_can addr;
      struct ifreq ifr;
      std::string bus_name = "can";
      if (virtual_)
      {
        bus_name = "vcan";
      }
      std::string can_channel_name = bus_name + std::to_string(i);
      sprintf(ifr.ifr_name, "%s", can_channel_name.c_str());

      ifr.ifr_name[IFNAMSIZ - 1] = '\0';
      ifr.ifr_ifindex = if_nametoindex(ifr.ifr_name);
      if (!ifr.ifr_ifindex)
      {
        RCLCPP_ERROR(this->get_logger(), "if_nametoindex %s error", ifr.ifr_name);
        throw(socket_can_exception::CONFIG_FAIL);
      }

      ioctl(s, SIOCGIFINDEX, &ifr);

      memset(&addr, 0, sizeof(addr));
      addr.can_family = AF_CAN;
      addr.can_ifindex = ifr.ifr_ifindex;

      if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0)
      {
        RCLCPP_ERROR(this->get_logger(), "bind error");
        throw(socket_can_exception::BIND_FAIL);
      }

      RCLCPP_INFO(this->get_logger(), "Bind %s", ifr.ifr_name);

      // Save
      socket_hnd_.emplace_back(s);
      addr_list_.emplace_back(addr);
    }

    catch (const socket_can_exception &ex)
    {
      switch (ex)
      {
      case socket_can_exception::OPEN_FAIL:
        RCLCPP_ERROR(this->get_logger(), "CAN channel %d : Device open error.", i);
        break;
      case socket_can_exception::CONFIG_FAIL:
        RCLCPP_ERROR(this->get_logger(), "CAN channel %d : Device setting error.", i);
        break;
      case socket_can_exception::BIND_FAIL:
        RCLCPP_ERROR(this->get_logger(), "CAN channel %d : Device setting error.", i);
        break;
      case socket_can_exception::RECV_ERR:
        RCLCPP_ERROR(this->get_logger(), "CAN channel %d : Device receive error.", i);
        break;
      case socket_can_exception::NO_MSG:
        RCLCPP_ERROR(this->get_logger(), "CAN channel %d : Device no message error.", i);
        break;
      case socket_can_exception::TIMEOUT:
        RCLCPP_ERROR(this->get_logger(), "CAN channel %d : Device timeout error.", i);
        break;
      }
      rclcpp::shutdown();
      return;
    }
    catch (...)
    {
      RCLCPP_ERROR(this->get_logger(), "CAN channel %d : UNKNOWN error.", i);
      rclcpp::shutdown();
      return;
    }
  }

  for (int32_t i = 0; i < device_channel_num_; ++i)
  {
    // Make receive thread
    receive_thread_list_.emplace_back(std::thread([this, i]()
                                                  { this->canReceiveThread(i); }));
    RCLCPP_INFO(this->get_logger(), "Start %d receive thread", i);
  }
} // end SocketCanDriver()

/*--------------------------------------------------------------------
 * ~SocketCanDriver()
 * Destructor.
 *------------------------------------------------------------------*/
SocketCanDriver::~SocketCanDriver()
{
  for (int i = 0; i < device_channel_num_; ++i)
  {
    if ((int)receive_thread_list_.size() > i)
    {
      receive_thread_list_[i].join();
      int hnd = socket_hnd_[i];
      close(hnd);
    }
  }
  delete socket_mtx_;
} // end ~SocketCanDriver()

/*--------------------------------------------------------------------
 * cantxCallback()
 * Callback function for subscriber.
 *------------------------------------------------------------------*/
// callback
void SocketCanDriver::cantxCallback(const ros_socket_can_driver::msg::CANFrame::SharedPtr msg)
{
  for (int i = 0; i < (int)can_tx_config_.msgs.size(); ++i)
  {
    if ((uint16_t)can_tx_config_.msgs[i].can_id == (uint16_t)msg->can_id &&
        (uint8_t)can_tx_config_.msgs[i].device_channel == (uint8_t)msg->device_channel)
    {
      // CAN TX
      struct can_frame frame;
      frame.can_id = (uint16_t)msg->can_id;
      frame.can_dlc = (uint8_t)msg->can_dlc;
      for (int j = 0; j < (uint8_t)msg->can_dlc; ++j)
      {
        frame.data[j] = msg->can_data[j];
      }
      // Send
      if (write(socket_hnd_[can_tx_config_.msgs[i].device_channel], &frame, CAN_MTU) != CAN_MTU)
      {
        RCLCPP_WARN(this->get_logger(), "Cannot send CAN data ID : %d", (int)can_tx_config_.msgs[i].can_id);
      }
      break;
    }
  }
}

void SocketCanDriver::canReceiveThread(uint8_t channel_num)
{
  rcutils_duration_value_t throttle_period = 10000; // スロットリングの時間間隔 (ここでは10秒)

  struct sockaddr_can addr = addr_list_[channel_num];
  int retval;
  struct mmsghdr msgs[BUFF_SIZE];
  struct iovec iovecs[BUFF_SIZE];
  struct can_frame frames[BUFF_SIZE];
  struct timespec timeout;
  memset(msgs, 0, sizeof(msgs));

  RCLCPP_INFO(this->get_logger(), "Start handle %d receive thread", channel_num);

  for (int i = 0; i < BUFF_SIZE; i++)
  {
		struct iovec *iovec = &iovecs[i];
		struct mmsghdr *msg = &msgs[i];

		msg->msg_hdr.msg_iov = iovec;
		msg->msg_hdr.msg_iovlen = 1;

		iovec->iov_base = &frames[i];
		iovec->iov_len = sizeof(can_frame);
  }

  timeout.tv_sec = 0;
  timeout.tv_nsec = 0;
  
  rclcpp::Clock system_clock(RCL_SYSTEM_TIME);

  while (rclcpp::ok())
  {
    // Read CAN Data
    try
    {
      //int retval = recvmmsg(socket_hnd_[channel_num], msgs, BUFF_SIZE, MSG_DONTWAIT, NULL);
      int retval = recvmmsg(socket_hnd_[channel_num], msgs, BUFF_SIZE, 0, &timeout);

      if (retval < 0)
      {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this, throttle_period, "read error %d", channel_num);
        continue;
      }      
      auto time_now = system_clock.now();

      // Extract seconds and nanoseconds from the time_now
      auto nanoseconds = time_now.nanoseconds();
      for (int i = 0; i < retval; i++)
      {
        char buf[CAN_MAX_DLEN]; 
        int id = frames[i].can_id;
        int dlc = frames[i].can_dlc;

        // Processing data
        for (int i = 0; i < (int)can_rx_config_.msgs.size(); ++i)
        {
          if (can_rx_config_.msgs[i].can_id == id &&
              can_rx_config_.msgs[i].device_channel == channel_num)
          {
            ros_socket_can_driver::msg::CANFrame out_frame;
            out_frame.device_channel = channel_num;   
            out_frame.header.stamp.sec = nanoseconds / 1000000000; // converting nanoseconds to seconds
            out_frame.header.stamp.nanosec = nanoseconds % 1000000000; // the remaining nanoseconds
            out_frame.header.frame_id = "base_link";
            out_frame.can_id = id;
            out_frame.can_dlc = dlc;
            for (int k = 0; k < (int)dlc; ++k)
            {
              out_frame.can_data.push_back(frames[i].data[k]);
            }
            socket_can_rx_pub_->publish(out_frame);
          }
        }
      }
    }
    catch (...)
    {
      RCLCPP_ERROR(this->get_logger(), "CAN ch%d: Unexpected Error.", channel_num);
      rclcpp::shutdown();
    }
  }
  RCLCPP_WARN(this->get_logger(), "Exit receive thread %d", channel_num);
}
