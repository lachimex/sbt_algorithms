#include "gps_converter.hpp"

#define EARTH_RADIUS 6371000  // metres
#define PI 3.141592653589793238462643383279502884197

using std::placeholders::_1;
using std::placeholders::_2;

double deg2rad(double degree) { return (degree * (PI / 180)); }

double HaversineDist(double lat0, double lon0, double lat1, double lon1) {
  double dlat = deg2rad(lat1 - lat0);
  double dlon = deg2rad(lon1 - lon0);
  double a = std::pow(std::sin(dlat / 2), 2) +
             std::cos(deg2rad(lat0)) * std::cos(deg2rad(lat1)) * std::pow(std::sin(dlon / 2), 2);
  const double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));
  return EARTH_RADIUS * c;
}

GpsConverter::GpsConverter() : Node("GpsConverter") {
  RCLCPP_INFO(this->get_logger(), "[GpsConverter] Node started");
  l1_gps_sub = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      "/gps/nav_sat_fix", 10, std::bind(&GpsConverter::Callback, this, _1));
  l2_gps_pub =
      this->create_publisher<l2_interfaces::msg::MetricPosition>("/gps/metric_position", 10);
  set_home_service_keyboard = this->create_service<l2_interfaces::srv::SetHome>(
      "/srv/set_home_keyboard_input", std::bind(&GpsConverter::SetHomeKeyboard, this, _1, _2));
  set_home_based_on_current_position = this->create_service<std_srvs::srv::Trigger>(
      "/srv/set_home_boat_curr_position",
      std::bind(&GpsConverter::SetHomeCurrPosition, this, _1, _2));
}

void GpsConverter::Callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
  if(!isBasePointSet){
    basePoint.latitude = msg->latitude;
    basePoint.longitude = msg->longitude;
    isBasePointSet = true;
    RCLCPP_INFO(this->get_logger(), "Current home position is latitude: %lf, longitude: %lf",
              basePoint.latitude, basePoint.longitude);
  }
  l2_interfaces::msg::MetricPosition gps_info;
  gps_info.distance =
      HaversineDist(basePoint.latitude, basePoint.longitude, msg->latitude, msg->longitude);
  gps_info.x =
      HaversineDist(basePoint.latitude, basePoint.longitude, basePoint.latitude, msg->longitude);
  gps_info.y =
      HaversineDist(basePoint.latitude, basePoint.longitude, msg->latitude, basePoint.longitude);
  if (basePoint.longitude > msg->longitude) {
    gps_info.x *= -1;
  }
  if (basePoint.latitude > msg->latitude) {
    gps_info.y *= -1;
  }
  currentPosition.latitude = msg->latitude;
  currentPosition.longitude = msg->longitude;
  l2_gps_pub->publish(gps_info);
}

void GpsConverter::SetHomeKeyboard(
    const std::shared_ptr<l2_interfaces::srv::SetHome::Request> request,
    std::shared_ptr<l2_interfaces::srv::SetHome::Response> response) {
  basePoint.latitude = request->latitude;
  basePoint.longitude = request->longitude;
  response->if_home_changed = true;
  RCLCPP_INFO(this->get_logger(), "Home position set to latitude: %lf, longitude: %lf",
              request->latitude, request->longitude);
}

void GpsConverter::SetHomeCurrPosition(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  basePoint.latitude = currentPosition.latitude;
  basePoint.longitude = currentPosition.longitude;
  response->success = true;
  response->message = "Changed home to: " + std::to_string(basePoint.latitude) + " " +
                      std::to_string(basePoint.longitude);
  RCLCPP_INFO(this->get_logger(), "Current home position is latitude: %lf, longitude: %lf",
              basePoint.latitude, basePoint.longitude);
}