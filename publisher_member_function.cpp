#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

using namespace std::chrono_literals;

class CameraCalibrationPublisher : public rclcpp::Node
{
public:
  CameraCalibrationPublisher()
  : Node("camera_calibration_publisher"), count_(0)
  {
    camera_info_publisher_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("camera_info_topic", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&CameraCalibrationPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto camera_info_msg = sensor_msgs::msg::CameraInfo();
    
    // Mock camera calibration data
    camera_info_msg.header.stamp = this->now();
    camera_info_msg.width = 640;
    camera_info_msg.height = 480;
    camera_info_msg.distortion_model = "plumb_bob";
    
    // Mock camera matrix
    camera_info_msg.k[0] = 400.0;  // fx
    camera_info_msg.k[2] = 320.0;  // cx
    camera_info_msg.k[4] = 400.0;  // fy
    camera_info_msg.k[5] = 240.0;  // cy
    camera_info_msg.k[8] = 1.0;

    // Mock distortion coefficients
    camera_info_msg.d.resize(5, 0.0);

    camera_info_publisher_->publish(camera_info_msg);

    RCLCPP_INFO(this->get_logger(), "Published Camera Calibration Data");
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_publisher_;
  size_t count_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CameraCalibrationPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
