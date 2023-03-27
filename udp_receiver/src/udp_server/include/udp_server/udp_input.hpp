#pragma once
#include <memory>

/*
 * The class UdpInput is a wrapper for the class UdpInput::UdpInputImpl.
 * Basic steps for contacting through UDP are:
 *    1. Create ros::NodeHandle
 *    2. Create a udp server using boost::asio
 *    3. Receive data from udp server
 *    4. Parse data
 *    5. Publish data to /cmd_vel topic
 */

class UdpInput {
 public:
  UdpInput(int argc, char** argv);
  ~UdpInput();

#pragma region Deleted
  UdpInput(const UdpInput&) = delete;
  UdpInput& operator=(const UdpInput&) = delete;
  UdpInput(UdpInput&&) = delete;
  UdpInput& operator=(UdpInput&&) = delete;
#pragma endregion

  void run();

 private:
  class UdpInputImpl;
  std::unique_ptr<UdpInputImpl> impl_;

 public:
  static UdpInput* create(int argc, char** argv);
};
