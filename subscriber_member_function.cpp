#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

using std::placeholders::_1;

class CameraCalibrationSubscriber : public rclcpp::Node
{
public:
  CameraCalibrationSubscriber()
  : Node("camera_calibration_subscriber")
  {
    camera_info_subscription_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "camera_info_topic", 10, std::bind(&CameraCalibrationSubscriber::camera_info_callback, this, _1));
  }

private:
  void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) const
  { 
    RCLCPP_INFO(this->get_logger(), "Received Camera Calibration Data:");
    RCLCPP_INFO(this->get_logger(), "  Width: %d, Height: %d", msg->width, msg->height);
    RCLCPP_INFO(this->get_logger(), "  Distortion Model: %s", msg->distortion_model.c_str());
    RCLCPP_INFO(this->get_logger(), "  Camera Matrix (fx, cx, fy, cy): %.2f, %.2f, %.2f, %.2f", 
                msg->k[0], msg->k[2], msg->k[4], msg->k[5]);
    RCLCPP_INFO(this->get_logger(), "  Distortion Coefficients: %.2f, %.2f, %.2f, %.2f, %.2f", 
                msg->d[0], msg->d[1], msg->d[2], msg->d[3], msg->d[4]);
  }

  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscription_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CameraCalibrationSubscriber>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
