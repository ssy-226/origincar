#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>

#include <opencv2/opencv.hpp>
#include <zbar.h>

#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>

#include <functional>
#include <stdexcept>
#include <string>

class Qrcode : public rclcpp::Node
{
public:
  Qrcode() : Node("qr"), frame_count_(0)
  {
    this->declare_parameter<std::string>("rgb_topic", "/rgb/image_raw");
    rgb_topic_ = this->get_parameter("rgb_topic").as_string();

    rclcpp::QoS qos(1);
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

    subscriber_rgb_ = this->create_subscription<sensor_msgs::msg::Image>(
        rgb_topic_,
        qos,
        std::bind(&Qrcode::subscription_callback, this, std::placeholders::_1));

    scanner_.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);

    qrcode_number_publisher_ =
        this->create_publisher<std_msgs::msg::Int32>("/qrcode_number", 10);

    qrcode_text_publisher_ =
        this->create_publisher<std_msgs::msg::String>("/qrcode_text", 10);

    RCLCPP_INFO(
        this->get_logger(),
        "qrcode node started: subscribe %s, pub /qrcode_number and /qrcode_text",
        rgb_topic_.c_str());
  }

private:
  void subscription_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    if (!msg)
      return;

    cv::Mat bgr;
    try
    {
      bgr = cv_bridge::toCvShare(msg, "bgr8")->image;
    }
    catch (const cv_bridge::Exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "convert Aurora RGB image failed: %s", e.what());
      return;
    }

    if (bgr.empty())
      return;

    cv::Mat gray;
    cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);

    // ZBar directly processes the native Aurora RGB resolution (640x400).
    // Do not resize or interpolate the QR image.
    if (!gray.isContinuous())
    {
      gray = gray.clone();
    }

    frame_count_++;
    if (frame_count_ % 30 == 0)
    {
      RCLCPP_INFO(
          this->get_logger(),
          "received Aurora RGB image: %dx%d",
          gray.cols,
          gray.rows);
    }

    zbar::Image zbar_image(
        gray.cols,
        gray.rows,
        "Y800",
        gray.data,
        gray.cols * gray.rows);
    int result = scanner_.scan(zbar_image);

    if (result <= 0)
      return;

    for (zbar::Image::SymbolIterator symbol = zbar_image.symbol_begin();
         symbol != zbar_image.symbol_end();
         ++symbol)
    {
      std::string qr_data = symbol->get_data();

      std_msgs::msg::String qrcode_text_msg;
      qrcode_text_msg.data = qr_data;
      qrcode_text_publisher_->publish(qrcode_text_msg);

      std_msgs::msg::Int32 qrcode_number_msg;
      bool should_publish_number = true;

      if (qr_data == "ClockWise")
      {
        qrcode_number_msg.data = 3;
      }
      else if (qr_data == "AntiClockWise")
      {
        qrcode_number_msg.data = 4;
      }
      else
      {
        try
        {
          int number = std::stoi(qr_data);
          if (number >= 1 && number <= 9999)
          {
            qrcode_number_msg.data = (number % 2 == 0) ? 4 : 3;
          }
          else
          {
            RCLCPP_WARN(this->get_logger(), "recognized number out of range: %d", number);
            should_publish_number = false;
          }
        }
        catch (const std::invalid_argument &e)
        {
          RCLCPP_WARN(this->get_logger(), "unrecognized QR content: %s", qr_data.c_str());
          should_publish_number = false;
        }
      }

      if (should_publish_number)
      {
        qrcode_number_publisher_->publish(qrcode_number_msg);
        RCLCPP_INFO(
            this->get_logger(),
            "QR text: %s, publish /qrcode_number: %d",
            qr_data.c_str(),
            qrcode_number_msg.data);
      }
      else
      {
        RCLCPP_INFO(
            this->get_logger(),
            "QR text: %s, only publish /qrcode_text",
            qr_data.c_str());
      }
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber_rgb_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr qrcode_number_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr qrcode_text_publisher_;

  std::string rgb_topic_;
  zbar::ImageScanner scanner_;
  int frame_count_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Qrcode>());
  rclcpp::shutdown();
  return 0;
}
