#include "udp_server/udp_input.hpp"

#include <ros/ros.h>

#include <boost/asio.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/system/error_code.hpp>
#include <cstdint>
#include <cstring>

#include "ros/init.h"
#include "ros/publisher.h"
#include "ros/rate.h"
#include "udp_server/twist_generate.hpp"

// TODO: Add error handling
// TODO: Complete the data parsing and publishing

#define UDP_DEFAULT_PORT 9999
#define UDP_DEFAULT_BUFSIZE 1234
#define UDP_RATE 20

#define MAX_LINEAR_X 0.5
#define MAX_ANGULAR_Z 0.5

using boost::asio::ip::udp;

class UdpInput::UdpInputImpl {
 public:
  // TODO: Add error handling
  /**
   * @brief Construct a new Udp Input Impl object
   *        This constructor will initialize the ros node and udp server
   */
  UdpInputImpl(int argc, char** argv)
      : nh_private_(ros::NodeHandle("~")),
        ip_(argv[1]),
        port_(boost::lexical_cast<int16_t>(argv[2])),
        socket_(
            context_,
            udp::endpoint(boost::asio::ip::address::from_string(ip_), port_)) {
    ROS_INFO("UDP_SERVER STARTED ON %s:%d", ip_, port_);
    pub_ = nh_private_.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1);
  }

  /**
   * @brief Run the udp server
   *        This function will receive data from udp client, parse data like {
   * x:1, z:1 } and publish data to /cmd_vel topic
   */
  void run() {
    ros::Rate rate(UDP_RATE);

    while (ros::ok()) {
      // Receive data from udp server
      udp::endpoint sender_endpoint;
      size_t len = socket_.receive_from(boost::asio::buffer(data_),
                                        sender_endpoint, 0, error);
      if (error && error != boost::asio::error::message_size) {
        ROS_ERROR("ERROR WHILE RECEIVING DATA: %s", error.message().c_str());
        error.clear();
        continue;
      }

      // Parse data
      TwistGenerate twist_gen{max_linear_x_, max_angular_z_, data_};
      if (twist_gen.error()) {
        ROS_ERROR("ERROR WHILE PARSING DATA");
        continue;
      }

      // Publish data to /cmd_vel topic
      ROS_INFO("PUBLISHING: %f, %f", twist_gen.twist().linear.x,
               twist_gen.twist().angular.z);
      pub_.publish(twist_gen.twist());
      socket_.send_to(boost::asio::buffer(data_), sender_endpoint);

      rate.sleep();
      ros::spinOnce();
      memset(data_, 0, UDP_DEFAULT_BUFSIZE);
    }
  }

  ~UdpInputImpl() {
    socket_.close();
    context_.stop();
  };
#pragma region Deleted
  UdpInputImpl(const UdpInputImpl&) = delete;
  UdpInputImpl& operator=(const UdpInputImpl&) = delete;
  UdpInputImpl(UdpInputImpl&&) = delete;
  UdpInputImpl& operator=(UdpInputImpl&&) = delete;
#pragma endregion

 private:
  ros::NodeHandle nh_private_;
  ros::Publisher pub_;

  const char* ip_;
  int16_t port_;

  double max_linear_x_{MAX_LINEAR_X};
  double max_angular_z_{MAX_ANGULAR_Z};

  boost::asio::io_context context_;
  udp::socket socket_;
  boost::system::error_code error;
  char data_[UDP_DEFAULT_BUFSIZE]{};
};

void UdpInput::run() { impl_->run(); }

UdpInput::UdpInput(int argc, char** argv)
    : impl_(new UdpInputImpl(argc, argv)) {}
UdpInput::~UdpInput() = default;

UdpInput* UdpInput::create(int argc, char** argv) {
  ros::init(argc, argv, "udp_input");
  return new UdpInput(argc, argv);
}
