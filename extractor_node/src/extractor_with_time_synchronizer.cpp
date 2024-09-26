#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "cae_microphone_array/msg/audio_stream.hpp"
#include "extractor_node/msg/av_reader.hpp"
#include "message_filters/sync_policies/approximate_time.h"
#include "sensor_msgs/msg/image.hpp"

class ExtractorNode : public rclcpp::Node
{
public:
  ExtractorNode() : Node("extractor")
  {
    //set the QoS for the image topic
    rclcpp::QoS image_qos(rclcpp::KeepLast(10));
    image_qos.best_effort();


    // // subscribe to the image topic 
    // image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
    //   "/cae_micarray/images/pict", image_qos, std::bind(&ExtractorNode::image_callback, this, std::placeholders::_1));

    // // subscribe to the audio topic
    // audio_subscription_ = this->create_subscription<cae_microphone_array::msg::AudioStream>(
    //   "/cae_micarray/audio/array", 10, std::bind(&ExtractorNode::audio_callback, this, std::placeholders::_1));

    // subscribe to the image topic 
     auto image_subscriber = this->create_subscription<sensor_msgs::msg::Image>(
            "/cae_micarray/images/pict", image_qos,
            [this](const sensor_msgs::msg::Image::SharedPtr msg) {
            });

    image_subscription_ .subscribe(this, "/cae_micarray/images/pict");

    // subscribe to the audio topic
    audio_subscription_.subscribe(this, "/cae_micarray/audio/array");



    // create a publisher for the audio and image data
    av_publisher =  this->create_publisher<extractor_node::msg::AvReader>("/extractor/av_message", 10);

    //create a time sychronizer for the audio and image data
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, cae_microphone_array::msg::AudioStream> MySyncPolicy;
    sync_ = std::make_shared<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, cae_microphone_array::msg::AudioStream>>(10);
    sync_->connectInput(image_subscription_, audio_subscription_);

    // 注册同步回调函数
    sync_->registerCallback(std::bind(&ExtractorNode::av_message_callback_, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "Node initialized, waiting for messages on image_topic and audio_stream_topic.");
  }

private:

//   // delcare the image and audio subscription
//   rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
//   rclcpp::Subscription<cae_microphone_array::msg::AudioStream>::SharedPtr audio_subscription_;

//   // delcare the image and audio publisher
//   rclcpp::Publisher<extractor_node::msg::AvReader>::SharedPtr av_publisher;

//   //delcare a vector to store the audio data
//   std::vector<double> current_mic_data;

//   void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
//   {
//     RCLCPP_INFO(this->get_logger(), "Image Recieved");

//     // check if the audio data is available
//     if (current_mic_data.empty())
//     {
//       RCLCPP_INFO(this->get_logger(), "No audio data available");
//       return;
//     }
//     else
//     {
//       extractor_node::msg::AvReader av_reader;
//       av_reader.header = msg->header;  
//       av_reader.image = *msg;
//       av_reader.audio= current_mic_data;
//       av_publisher->publish(av_reader);
//     }
//   }

//   void audio_callback(const cae_microphone_array::msg::AudioStream::SharedPtr msg)
//   {
//     RCLCPP_INFO(this->get_logger(), "Audio Recieved");
    
//     auto data = std::vector<uint8_t>(msg->data.begin(), msg->data.end());
//     std::size_t length = data.size();
//     std::size_t num_doubles = length / sizeof(double);

//     // transfer the audio data to a double array
//     std::vector<double> double_data(num_doubles);
//     std::memcpy(double_data.data(), data.data(), length);
//     std::cout << "Vector length: " << num_doubles << std::endl;

//     current_mic_data = double_data;
//   }

// };

  void av_message_callback_(const sensor_msgs::msg::Image::ConstSharedPtr& image, const cae_microphone_array::msg::AudioStream::ConstSharedPtr& audio)
      {
          RCLCPP_INFO(this->get_logger(), "Received synchronized messages");
      }

  // subscribe the message with message filter
  message_filters::Subscriber<sensor_msgs::msg::Image> image_subscription_;
  message_filters::Subscriber<cae_microphone_array::msg::AudioStream> audio_subscription_;

  // create the time synchronizer
  std::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, cae_microphone_array::msg::AudioStream>> sync_;
    
  rclcpp::Publisher<extractor_node::msg::AvReader>::SharedPtr av_publisher;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ExtractorNode>());
  rclcpp::shutdown();
  return 0;
}
