#include "udp_server/twist_generate.hpp"

#include <ros/ros.h>

#include "nlohmann/json.hpp"

/*
 * Clinet side will send data in the following format:
 * { x: 0.1, z: 0.2 }
 * x [-1, 1] is the linear velocity in x axis
 * z [-1, 1] is the angular velocity in z axis
 *
 * TwistGenerate will parse the data and generate a Twist message
 */

TwistGenerate::TwistGenerate(double max_linear_x, double max_angular_z,
                             const char* buffer) {
  // Parse data
  try {
    auto json = nlohmann::json::parse(buffer);
    twist_.linear.x = json["x"].get<double>() * max_linear_x;
    twist_.angular.z = json["z"].get<double>() * max_angular_z;
  } catch (const std::exception& e) {
    ROS_ERROR("ERROR WHILE PARSING DATA: %s", e.what());
    error_ = true;
  }
}
