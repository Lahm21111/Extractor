#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "cae_microphone_array/msg/audio_stream.hpp"
#include "sensor_msgs/msg/image.hpp"

class ExtractorNode : public rclcpp::Node
{
public:
  ExtractorNode() : Node("extractor")
  {
    //set the QoS for the image topic
    rclcpp::QoS image_qos(rclcpp::KeepLast(10));
    image_qos.best_effort();

    // subscribe to the image topic 
    image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/cae_micarray/images/pict", image_qos, std::bind(&ExtractorNode::image_callback, this, std::placeholders::_1));

    // subscribe to the audio topic
    audio_subscription_ = this->create_subscription<cae_microphone_array::msg::AudioStream>(
      "/cae_micarray/audio/array", 10, std::bind(&ExtractorNode::audio_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Node initialized, waiting for messages on image_topic and audio_stream_topic.");
  }

private:

  // delcare the image and audio subscription
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
  rclcpp::Subscription<cae_microphone_array::msg::AudioStream>::SharedPtr audio_subscription_;

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Image Recieved");
  }

  void audio_callback(const cae_microphone_array::msg::AudioStream::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Audio Recieved");
  }

};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ExtractorNode>());
  rclcpp::shutdown();
  return 0;
}
