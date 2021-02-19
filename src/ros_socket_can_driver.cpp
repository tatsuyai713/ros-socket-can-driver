#include "ros_socket_can_driver.h"

#define BUFF_SIZE 256

void operator>>(const YAML::Node &node, CANMsg &msg)
{
  msg.name = node["name"].as<std::string>();
  msg.channel = (uint8_t)node["channel"].as<int>();
  msg.can_id = (uint16_t)node["can_id"].as<int>();
  msg.can_dlc = (uint8_t)node["can_dlc"].as<int>();
  msg.can_data = node["can_data"].as<std::vector<uint8_t>>();
}

CANInfo loadYMLFile(std::string ymlpath)
{
  CANInfo info;
  try
  {
    YAML::Node node = YAML::LoadFile(ymlpath);
    CANMsg msg;
    for (int i = 0; i < (int)node.size(); i++)
    {
      node[i] >> msg;
      std::cout << msg.name << std::endl;
      info.msgs.push_back(msg);
    }
  }
  catch (YAML::Exception &e)
  {
    std::cerr << e.what() << std::endl;
  }
  return info;
}

SocketCanDriver::SocketCanDriver()
    : nh_(""), pnh_("~")
{

  // ROS
  socket_can_tx_sub_ = nh_.subscribe("/vehicle/can_tx", 10, &SocketCanDriver::cantxCallback, this);
  socket_can_rx_pub_ = nh_.advertise<ros_socket_can_driver::CANFrame>("/vehicle/can_rx", 10);

  // parameters
  pnh_.param("max_ch_num", max_ch_num_, 1);
  pnh_.param("bitrate", bitrate_, 500000);
  pnh_.param("virtual", virtual_, false);
  pnh_.param("can_tx_setting", can_tx_setting_, std::string("can_tx.yaml"));
  pnh_.param("can_rx_setting", can_rx_setting_, std::string("can_rx.yaml"));

  // Get Parameters from YAML file
  tx_info_.msgs.clear();
  std::string can_tx_file;
  can_tx_file = ros::package::getPath("ros_socket_can_driver") + "/config/" + can_tx_setting_;
  tx_info_ = loadYMLFile(can_tx_file);

  rx_info_.msgs.clear();
  std::string can_rx_file;
  can_rx_file = ros::package::getPath("ros_socket_can_driver") + "/config/" + can_rx_setting_;
  rx_info_ = loadYMLFile(can_rx_file);

  receive_thread_list_.clear();
  socket_mtx_ = new std::mutex[max_ch_num_];

  // socket can initialize
  for (int i = 0; i < max_ch_num_; ++i)
  {
    try
    {
      // Open can socket
      int s;
      if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
      {
        ROS_ERROR("socket open error");
        throw(socket_can_exception::OPEN_FAIL);
      }
      ROS_INFO("Open %d channel", i);

      int rcvbuf_size = BUFF_SIZE;
      if (setsockopt(s, SOL_SOCKET, SO_RCVBUF,
                     &rcvbuf_size, sizeof(rcvbuf_size)) < 0)
      {
        ROS_ERROR("setsockopt SO_RCVBUF");
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
        ROS_ERROR("if_nametoindex %s error", ifr.ifr_name);
        throw(socket_can_exception::CONFIG_FAIL);
      }

      ioctl(s, SIOCGIFINDEX, &ifr);

      memset(&addr, 0, sizeof(addr));
      addr.can_family = AF_CAN;
      addr.can_ifindex = ifr.ifr_ifindex;

      if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0)
      {
        ROS_ERROR("bind error");
        throw(socket_can_exception::BIND_FAIL);
      }

      ROS_INFO("Bind %s", ifr.ifr_name);

      // Save
      socket_hnd_.emplace_back(s);
      addr_list_.emplace_back(addr);

      // Thread
      receive_thread_list_.emplace_back(std::thread([this, i]() { this->can_receive_thread(i); }));
      ROS_INFO("Make %d thread", i);
    }

    catch (const socket_can_exception &ex)
    {
      switch (ex)
      {
      case socket_can_exception::OPEN_FAIL:
        ROS_ERROR("CAN channel %d : Device open error.", i);
        break;
      case socket_can_exception::CONFIG_FAIL:
        ROS_ERROR("CAN channel %d : Device setting error.", i);
        break;
      case socket_can_exception::BIND_FAIL:
        ROS_ERROR("CAN channel %d : Device setting error.", i);
        break;
      case socket_can_exception::RECV_ERR:
        ROS_ERROR("CAN channel %d : Device receive error.", i);
        break;
      case socket_can_exception::NO_MSG:
        ROS_ERROR("CAN channel %d : Device no message error.", i);
        break;
      case socket_can_exception::TIMEOUT:
        ROS_ERROR("CAN channel %d : Device timeout error.", i);
        break;
      }
      ros::shutdown();
      return;
    }
    catch (...)
    {
      ROS_ERROR("CAN channel %d : UNKNOWN error.", i);
      ros::shutdown();
      return;
    }
  }

} // end SocketCanDriver()

/*--------------------------------------------------------------------
 * ~SocketCanDriver()
 * Destructor.
 *------------------------------------------------------------------*/
SocketCanDriver::~SocketCanDriver()
{
  for (int i = 0; i < max_ch_num_; ++i)
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
void SocketCanDriver::cantxCallback(const ros_socket_can_driver::CANFrame::ConstPtr &can_frame)
{
  for (int i = 0; i < (int)tx_info_.msgs.size(); ++i)
  {
    if ((uint16_t)tx_info_.msgs[i].can_id == (uint16_t)can_frame->can_id &&
        (uint8_t)tx_info_.msgs[i].can_dlc == (uint8_t)can_frame->can_dlc &&
        (uint8_t)tx_info_.msgs[i].channel == (uint8_t)can_frame->can_channel)
    {
      tx_info_.msgs[i].can_data.clear();
      for (int j = 0; j < tx_info_.msgs[i].can_dlc; ++j)
      {
        tx_info_.msgs[i].can_data.push_back(can_frame->can_data[j]);
      }
      // CAN TX
      struct can_frame frame;
      frame.can_id = tx_info_.msgs[i].can_id;
      frame.can_dlc = tx_info_.msgs[i].can_dlc;
      for (int j = 0; j < tx_info_.msgs[i].can_dlc; ++j)
      {
        frame.data[j] = tx_info_.msgs[i].can_data[j];
      }
      // Send
      if (write(socket_hnd_[tx_info_.msgs[i].channel], &frame, CAN_MTU) != CAN_MTU)
      {
        ROS_WARN("Cannot send CAN data ID : %d", (int)tx_info_.msgs[i].can_id);
      }
      break;
    }
  }
}

void SocketCanDriver::can_receive_thread(unsigned char ch_num)
{
  struct sockaddr_can addr = addr_list_[ch_num];
  ros::Time stamp;
  int retval;
  struct mmsghdr msgs[BUFF_SIZE];
  struct iovec iovecs[BUFF_SIZE];
  struct can_frame frames[BUFF_SIZE];
  struct timespec timeout;
  memset(msgs, 0, sizeof(msgs));

  ROS_INFO("Start handle %d receive thread", ch_num);

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

  while (ros::ok())
  {
    // Read CAN Data
    try
    {
      //int retval = recvmmsg(socket_hnd_[ch_num], msgs, BUFF_SIZE, MSG_DONTWAIT, NULL);
      int retval = recvmmsg(socket_hnd_[ch_num], msgs, BUFF_SIZE, 0, &timeout);

      if (retval < 0)
      {
        ROS_WARN_DELAYED_THROTTLE(10, "read error %d", ch_num);
        continue;
      }
      for (int i = 0; i < retval; i++)
      {
        char buf[CAN_MAX_DLEN]; 
        int id = frames[i].can_id;
        int dlc = frames[i].can_dlc;
        stamp = ros::Time::now();

        // Processing data
        for (int j = 0; j < (int)rx_info_.msgs.size(); ++j)
        {
          if (rx_info_.msgs[j].can_id == id &&
              rx_info_.msgs[j].channel == ch_num &&
              rx_info_.msgs[j].can_dlc == dlc)
          {
            ros_socket_can_driver::CANFrame out_frame;
            out_frame.can_msg_name = rx_info_.msgs[j].name;
            out_frame.can_channel = ch_num;
            out_frame.header.stamp = stamp;
            out_frame.header.frame_id = "/can/data";
            out_frame.can_id = id;
            out_frame.can_dlc = dlc;
            for (int k = 0; k < (int)dlc; ++k)
            {
              out_frame.can_data.push_back(frames[i].data[k]);
            }
            socket_can_rx_pub_.publish(out_frame);
          }
        }
      }
    }
    catch (...)
    {
      ROS_ERROR("CAN ch%d: Unexpected Error.", ch_num);
      ros::shutdown();
    }
  }
  ROS_WARN("Exit receive thread %d", ch_num);
}

/*--------------------------------------------------------------------
 * main()
 * Main function to set up ROS node.
 *------------------------------------------------------------------*/
int32_t main(int32_t argc, char **argv)
{
  // Setup ROS.
  ros::init(argc, argv, "ros_socket_can_driver_node");
  SocketCanDriver node;

  ros::spin();
  std::cerr << "\nros_socket_can_driver_node: Exiting...\n";
  ros::shutdown();
  return (EXIT_SUCCESS);
} // end main()
