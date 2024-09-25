#include "socket_can_driver_node.h"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SocketCanDriver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}