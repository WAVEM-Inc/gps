// nav_sat_fix_listener.cpp: Listens to NavSatFix data on the /ublox/fix topic and logs the messages.

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

class NavSatFixListener : public rclcpp::Node {
 public:
  NavSatFixListener() : Node("nav_sat_fix_listener") {
    listener_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/ublox/fix", 10, std::bind(&NavSatFixListener::NavSatFixCallback, this, std::placeholders::_1));
  }

 private:
  void NavSatFixCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) const {
    RCLCPP_INFO(this->get_logger(), "Received NavSatFix Data:\n- Latitude: %f\n- Longitude: %f\n- Altitude: %f\n- Covariance Type: %u",
                msg->latitude, msg->longitude, msg->altitude, msg->position_covariance_type);
    // Additional message processing can be done here.
  }

  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr listener_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavSatFixListener>());
  rclcpp::shutdown();
  return 0;
}

