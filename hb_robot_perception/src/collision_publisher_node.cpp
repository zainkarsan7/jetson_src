#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>

#include <mutex>
#include <memory>

class CollisionDepthAdapter : public rclcpp::Node
{
public:
  CollisionDepthAdapter()
  : Node("collision_depth_adapter")
  {
    declare_parameter<std::string>(
        "depth_topic",
        "/k4a/depth_to_rgb/image_raw");

    declare_parameter<std::string>(
        "camera_info_topic",
        "/k4a/depth_to_rgb/camera_info");  // VERIFY THIS TOPIC

    declare_parameter<std::string>(
        "output_image_topic",
        "/collision/depth/image_raw");

    declare_parameter<std::string>(
        "output_info_topic",
        "/collision/depth/camera_info");

    declare_parameter<int>("decimation", 4);
    declare_parameter<double>("near_clip", 0.1);
    declare_parameter<double>("far_clip", 1.5);
    
    depth_topic_ = get_parameter("depth_topic").as_string();
    info_topic_ = get_parameter("camera_info_topic").as_string();
    output_image_topic_ = get_parameter("output_image_topic").as_string();
    output_info_topic_ = get_parameter("output_info_topic").as_string();
    decimation_ = get_parameter("decimation").as_int();
    near_clip_ = static_cast<float>(get_parameter("near_clip").as_double());
    far_clip_ = static_cast<float>(get_parameter("far_clip").as_double());


    if (decimation_ < 1)
      throw std::runtime_error("decimation must be >= 1");

    auto qos = rclcpp::SensorDataQoS().keep_last(1);

    depth_sub_ =
        create_subscription<sensor_msgs::msg::Image>(
            depth_topic_,
            qos,
            std::bind(
                &CollisionDepthAdapter::depthCallback,
                this,
                std::placeholders::_1));

    info_sub_ =
        create_subscription<sensor_msgs::msg::CameraInfo>(
            info_topic_,
            qos,
            std::bind(
                &CollisionDepthAdapter::infoCallback,
                this,
                std::placeholders::_1));

    image_pub_ =
        create_publisher<sensor_msgs::msg::Image>(
            output_image_topic_,
            qos);

    info_pub_ =
        create_publisher<sensor_msgs::msg::CameraInfo>(
            output_info_topic_,
            qos);

    RCLCPP_INFO(
        get_logger(),
        "Collision depth adapter: %s + %s -> %s, decimation=%d",
        depth_topic_.c_str(),
        info_topic_.c_str(),
        output_image_topic_.c_str(),
        decimation_);
  }

private:
  void infoCallback(
      const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(info_mutex_);
    latest_info_ = msg;
  }

  void depthCallback(
      const sensor_msgs::msg::Image::ConstSharedPtr depth_msg)
  {
    sensor_msgs::msg::CameraInfo::ConstSharedPtr source_info;

    {
      std::lock_guard<std::mutex> lock(info_mutex_);
      source_info = latest_info_;
    }

    if (!source_info)
    {
      RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "Waiting for CameraInfo");
      return;
    }

    // Registered Azure Kinect depth is expected to be 32FC1 here.
    if (depth_msg->encoding !=
        sensor_msgs::image_encodings::TYPE_32FC1)
    {
      RCLCPP_ERROR_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "Expected 32FC1 depth, got '%s'",
          depth_msg->encoding.c_str());
      return;
    }

    cv_bridge::CvImageConstPtr depth_cv;

    try
    {
      depth_cv = cv_bridge::toCvShare(
          depth_msg,
          sensor_msgs::image_encodings::TYPE_32FC1);
    }
    catch (const cv_bridge::Exception& e)
    {
      RCLCPP_ERROR(
          get_logger(),
          "cv_bridge failed: %s",
          e.what());
      return;
    }

    const cv::Mat& src = depth_cv->image;

    const int S = decimation_;

    // We sample exactly source pixels:
    // 0, S, 2S, ...
    const int out_rows = (src.rows + S - 1) / S;
    const int out_cols = (src.cols + S - 1) / S;

    cv::Mat dst(out_rows, out_cols, CV_32FC1);

    for (int v_out = 0, v = 0;
         v < src.rows;
         ++v_out, v += S)
    {
      const float* src_row = src.ptr<float>(v);
      float* dst_row = dst.ptr<float>(v_out);

      for (int u_out = 0, u = 0;
           u < src.cols;
           ++u_out, u += S)
      {
        float z = src_row[u];
        if(!std::isfinite(z) || z<near_clip_ || z>far_clip_){
          dst_row[u_out] = std::numeric_limits<float>::quiet_NaN();
        }
        else{
          dst_row[u_out] = src_row[u];
        }

        
      }
    }

    // ----- Output depth image -----

    std_msgs::msg::Header header = depth_msg->header;

    auto out_image =
        cv_bridge::CvImage(
            header,
            sensor_msgs::image_encodings::TYPE_32FC1,
            dst)
            .toImageMsg();

    // ----- Output CameraInfo -----

    sensor_msgs::msg::CameraInfo out_info = *source_info;

    // This is the important synchronization repair.
    out_info.header.stamp = depth_msg->header.stamp;

    // Both messages must describe the same camera coordinate frame.
    out_info.header.frame_id = depth_msg->header.frame_id;

    out_info.width = out_cols;
    out_info.height = out_rows;

    const double scale = 1.0 / static_cast<double>(S);

    // K =
    // [ fx  0 cx ]
    // [  0 fy cy ]
    // [  0  0  1 ]
    out_info.k[0] *= scale; // fx
    out_info.k[2] *= scale; // cx
    out_info.k[4] *= scale; // fy
    out_info.k[5] *= scale; // cy

    // P =
    // [ fx'  0 cx' Tx ]
    // [  0  fy' cy' Ty ]
    // [  0   0   1   0 ]
    //
    // Scale all pixel-coordinate quantities.
    out_info.p[0] *= scale;
    out_info.p[2] *= scale;
    out_info.p[3] *= scale;

    out_info.p[5] *= scale;
    out_info.p[6] *= scale;
    out_info.p[7] *= scale;

    // We're publishing a genuinely resized image rather than describing
    // sensor-side binning/cropping.
    out_info.binning_x = 0;
    out_info.binning_y = 0;

    out_info.roi.x_offset = 0;
    out_info.roi.y_offset = 0;
    out_info.roi.width = 0;
    out_info.roi.height = 0;
    out_info.roi.do_rectify = false;
    RCLCPP_INFO(
    get_logger(),
    "depth %ux%u -> %ux%u | "
    "fx %.2f -> %.2f, fy %.2f -> %.2f, "
    "cx %.2f -> %.2f, cy %.2f -> %.2f",
    depth_msg->width,
    depth_msg->height,
    out_info.width,
    out_info.height,
    source_info->k[0], out_info.k[0],
    source_info->k[4], out_info.k[4],
    source_info->k[2], out_info.k[2],
    source_info->k[5], out_info.k[5]);
    //
    // Publish CameraInfo first. Both messages carry the exact same stamp.
    //
    info_pub_->publish(out_info);
    image_pub_->publish(*out_image);

    RCLCPP_DEBUG_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Depth %ux%u -> %dx%d, stamp %.3f",
        depth_msg->width,
        depth_msg->height,
        out_cols,
        out_rows,
        rclcpp::Time(depth_msg->header.stamp).seconds());
  }

  std::string depth_topic_;
  std::string info_topic_;
  std::string output_image_topic_;
  std::string output_info_topic_;
  float near_clip_{0.05};
  float far_clip_{1.5};
  int decimation_{4};

  std::mutex info_mutex_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr latest_info_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(
      std::make_shared<CollisionDepthAdapter>());

  rclcpp::shutdown();
  return 0;
}