#ifndef GPS_CONVERTER_HPP_
#define GPS_CONVERTER_HPP_

#include "l2_interfaces/msg/metric_position.hpp"
#include "l2_interfaces/srv/set_home.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_srvs/srv/trigger.hpp"

struct NavSatPosition {
  double longitude;
  double latitude;

  NavSatPosition() : longitude(0), latitude(0){};
};

class GpsConverter : public rclcpp::Node {
public:
  GpsConverter();

private:
  NavSatPosition basePoint{};
  NavSatPosition currentPosition;
  bool isBasePointSet = false;

  void Callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);

  void SetHomeKeyboard(const std::shared_ptr<l2_interfaces::srv::SetHome::Request> request,
                       std::shared_ptr<l2_interfaces::srv::SetHome::Response> response);

  void SetHomeCurrPosition(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                           std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr l1_gps_sub;
  rclcpp::Publisher<l2_interfaces::msg::MetricPosition>::SharedPtr l2_gps_pub;
  rclcpp::Service<l2_interfaces::srv::SetHome>::SharedPtr set_home_service_keyboard;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr set_home_based_on_current_position;
};

#endif  // GPS_CONVERTER_HPP