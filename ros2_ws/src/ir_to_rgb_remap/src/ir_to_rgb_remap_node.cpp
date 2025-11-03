#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>

namespace ir_to_rgb_remap {

class IrToRgbRemapNode : public rclcpp::Node {
public:
  explicit IrToRgbRemapNode(const rclcpp::NodeOptions& options)
  : rclcpp::Node("ir_to_rgb_remap", options) {
    in_image_ = this->declare_parameter<std::string>("in_image", "/camera/infra1/image_rect_raw");
    out_image_ = this->declare_parameter<std::string>("out_image", "/image_rect");
    in_info_  = this->declare_parameter<std::string>("in_camera_info", "/camera/infra1/camera_info");
    out_info_ = this->declare_parameter<std::string>("out_camera_info", "/camera_info_rect");

    img_pub_  = this->create_publisher<sensor_msgs::msg::Image>(out_image_, rclcpp::SensorDataQoS());
    info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(out_info_, 10);

    img_sub_  = this->create_subscription<sensor_msgs::msg::Image>(
      in_image_, rclcpp::SensorDataQoS(),
      std::bind(&IrToRgbRemapNode::onImage, this, std::placeholders::_1));

    info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      in_info_, 10, [this](sensor_msgs::msg::CameraInfo::SharedPtr msg){
        info_pub_->publish(*msg);
      });

    RCLCPP_INFO(this->get_logger(), "IR→RGB remap: %s → %s, info %s → %s",
                in_image_.c_str(), out_image_.c_str(), in_info_.c_str(), out_info_.c_str());
  }

private:
  void onImage(const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
      cv_bridge::CvImageConstPtr in_cv = cv_bridge::toCvShare(msg, "mono8");
      cv::Mat rgb;
      cv::cvtColor(in_cv->image, rgb, cv::COLOR_GRAY2RGB);

      cv_bridge::CvImage out_cv;
      out_cv.header = msg->header;
      out_cv.encoding = "rgb8";
      out_cv.image = rgb;
      img_pub_->publish(*out_cv.toImageMsg());
    } catch (const std::exception& e) {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                            "IR→RGB conversion failed: %s", e.what());
    }
  }

  std::string in_image_, out_image_, in_info_, out_info_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
};

}  // namespace ir_to_rgb_remap

RCLCPP_COMPONENTS_REGISTER_NODE(ir_to_rgb_remap::IrToRgbRemapNode)
