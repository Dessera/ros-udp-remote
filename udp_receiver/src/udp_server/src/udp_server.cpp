#include "ros/init.h"
#include "udp_server/udp_input.hpp"

int main(int argc, char** argv) {
  if (argc != 3) {
    ROS_ERROR("Usage:rosrun udp_server udp_server <ip> <port>");
    return 1;
  }

  UdpInput* udp_input = UdpInput::create(argc, argv);
  udp_input->run();

  delete udp_input;
  return 0;
}