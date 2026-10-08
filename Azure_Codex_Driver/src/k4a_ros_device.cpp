// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

// Associated header
//
#include "azure_kinect_ros2_driver_codex/k4a_ros_device.h"


// System headers
//
#include <thread>
#include <algorithm>
#include <cstring>
#include <stdexcept>
#include <turbojpeg.h>

// Library headers
//
#include <angles/angles.h>
#include <cv_bridge/cv_bridge.h>
#include <k4a/k4a.hpp>

//#include <sensor_msgs/distortion_models.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>


// Project headers
//
#include "azure_kinect_ros2_driver_codex/k4a_ros_types.h"



using namespace sensor_msgs;
using namespace image_transport;
using namespace std;


K4AROS2Device::K4AROS2Device()
    : Node("k4a_ros2_node"),
      qos_(1),
      process_cloud_(false),
      last_capture_time_usec_(0),
      last_imu_time_usec_(0),
      imu_stream_end_of_file_(false)
{

  RCLCPP_INFO_STREAM(this->get_logger(), "Initializing " << this->get_name() << "...");

  // Collect ROS parameters from the param server or from the command line

  calibration_data_ = std::make_unique<K4ACalibrationTransformData>(this);


  // Declare the params
  std::string pSensorSn = this->declare_parameter<std::string>("sensor_sn", "");
  this->declare_parameter<bool>("depth_enabled", true);
  this->declare_parameter<std::string>("depth_mode", "NFOV_UNBINNED");
  this->declare_parameter<bool>("color_enabled", true);
  std::string pColorFormat = this->declare_parameter<std::string>("color_format", "bgra");
  this->declare_parameter<std::string>("color_resolution", "1536P");
  this->declare_parameter<int>("fps", 30);
  this->declare_parameter<bool>("point_cloud", false);
  this->declare_parameter<bool>("rgb_point_cloud", false);
  this->declare_parameter<bool>("point_cloud_in_depth_frame", true);
  this->declare_parameter<std::string>("tf_prefix", std::string());
  std::string pRecordingFile = this->declare_parameter<std::string>("recording_file", "");
  this->declare_parameter<bool>("recording_loop_enabled", false);
  this->declare_parameter<bool>("body_tracking_enabled", false);
  this->declare_parameter<int>("imu_rate_target", 100);
  this->declare_parameter<bool>("rescale_ir_to_mono8", false);
  this->declare_parameter<float>("ir_mono8_scaling_factor", 1.0f);;
  this->declare_parameter<int>("wired_sync_mode", 0);
  this->declare_parameter<int>("subordinate_delay_off_master_usec", 0);
  // Startup-only pipeline controls. Keep ROS color output unchanged while moving
  // JPEG decoding out of the SDK USB callback and into the bounded worker.
  rcl_interfaces::msg::ParameterDescriptor startup_only;
  startup_only.read_only = true;
  driver_color_decode_ = this->declare_parameter<bool>("driver_color_decode", true, startup_only);
  capture_timeout_ms_ = this->declare_parameter<int>("capture_timeout_ms", 1000, startup_only);
  recovery_max_attempts_ = this->declare_parameter<int>("recovery_max_attempts", 3, startup_only);
  recovery_backoff_ms_ = this->declare_parameter<int>("recovery_backoff_ms", 500, startup_only);
  max_capture_age_ms_ = this->declare_parameter<int>("max_capture_age_ms", 250, startup_only);
  if (capture_timeout_ms_ < 100 || recovery_max_attempts_ < 0 ||
      recovery_backoff_ms_ < 0 || max_capture_age_ms_ < 0)
  {
    throw std::invalid_argument("Invalid capture timeout, recovery, or frame age parameter");
  }

  // TODO: QoS
  //int pQosReliability = this->declare_parameter<int>("qos_reliability", 1);
  //int pQosDurability = this->declare_parameter<int>("qos_durability", 1);

  if (pRecordingFile != "")
  {
    // Replace the first "~"
    std::string home_dir = getenv("HOME");
    std::size_t pos = pRecordingFile.find("~");
    if (pos != std::string::npos)
    {
      pRecordingFile.replace(pos, 1, home_dir);
    }

    RCLCPP_INFO(this->get_logger(), "Node is started in playback mode");
    RCLCPP_INFO_STREAM(this->get_logger(), "Try to open recording file " << pRecordingFile);

    // Open recording file and print its length
    k4a_playback_handle_ = k4a::playback::open(pRecordingFile.c_str());
    auto recording_length = k4a_playback_handle_.get_recording_length();
    RCLCPP_INFO_STREAM(this->get_logger(), "Successfully opened recording file. Recording is " << recording_length.count() / 1000000
                                                                         << " seconds long");

    // Get the recordings configuration to overwrite node parameters
    k4a_record_configuration_t record_config = k4a_playback_handle_.get_record_configuration();

    // Overwrite fps param with recording configuration for a correct loop rate in the frame publisher thread
    switch (record_config.camera_fps)
    {
      case K4A_FRAMES_PER_SECOND_5:
        this->set_parameter(rclcpp::Parameter("fps", 5));
        break;
      case K4A_FRAMES_PER_SECOND_15:
        this->set_parameter(rclcpp::Parameter("fps", 15));
        break;
      case K4A_FRAMES_PER_SECOND_30:
        this->set_parameter(rclcpp::Parameter("fps", 30));
        break;
      default:
        break;
    };

    // Disable color if the recording has no color track
    //if (params_.color_enabled && !record_config.color_track_enabled)
    if (this->get_parameter("color_enabled").as_bool() && !record_config.color_track_enabled)
    {
      RCLCPP_WARN(this->get_logger(), "Disabling color and rgb_point_cloud because recording has no color track");
      this->set_parameter(rclcpp::Parameter("color_enabled", false));
      this->set_parameter(rclcpp::Parameter("point_cloud", false));
    }
    // This is necessary because at the moment there are only checks in place which use BgraPixel size
    else if (this->get_parameter("color_enabled").as_bool() && record_config.color_track_enabled)
    {
      if (pColorFormat == "jpeg" && record_config.color_format != K4A_IMAGE_FORMAT_COLOR_MJPG)
      {
        RCLCPP_FATAL(this->get_logger(), "Converting color images to K4A_IMAGE_FORMAT_COLOR_MJPG is not supported.");
        rclcpp::shutdown();
        return;
      }
      if (pColorFormat == "bgra" && record_config.color_format != K4A_IMAGE_FORMAT_COLOR_BGRA32)
      {
        k4a_playback_handle_.set_color_conversion(K4A_IMAGE_FORMAT_COLOR_BGRA32);
      }
    }

    // Disable depth if the recording has neither ir track nor depth track
    if (!record_config.ir_track_enabled && !record_config.depth_track_enabled)
    {
      if (this->get_parameter("depth_enabled").as_bool())
      {
        RCLCPP_WARN(this->get_logger(), "Disabling depth because recording has neither ir track nor depth track");
        this->set_parameter(rclcpp::Parameter("depth_enabled", false));
      }
    }

    // Disable depth if the recording has no depth track
    if (!record_config.depth_track_enabled)
    {
      RCLCPP_WARN(this->get_logger(), "No depth track in recording");
      if (this->get_parameter("point_cloud").as_bool())
      {
        RCLCPP_WARN(this->get_logger(), "Disabling point cloud because recording has no depth track");
        this->set_parameter(rclcpp::Parameter("point_cloud", false));
      }
      if (this->get_parameter("rgb_point_cloud").as_bool())
      {
        RCLCPP_WARN(this->get_logger(), "Disabling rgb point cloud because recording has no depth track");
        this->set_parameter(rclcpp::Parameter("rgb_point_cloud", false));
      }
    }
    RCLCPP_INFO(this->get_logger(), "Recording has depth track");

  }
  else
  {
    // Print all parameters
    RCLCPP_INFO(this->get_logger(), "K4A Parameters:");

    std::vector<std::string> param_names = {"sensor_sn", "depth_enabled", "depth_mode","color_enabled",
                                            "color_format", "color_resolution","fps", "point_cloud",
                                            "rgb_point_cloud", "point_cloud_in_depth_frame", "tf_prefix",
                                            "recording_file", "recording_loop_enabled", "imu_rate_target",
                                            "wired_sync_mode", "subordinate_delay_off_master_usec"};
    std::vector<rclcpp::Parameter> params = this->get_parameters(param_names);
    for (auto &param : params)
    {
      RCLCPP_INFO(this->get_logger(), "param name: %s, value: %s",
                  param.get_name().c_str(), param.value_to_string().c_str());
    }


    // Setup the K4A device
    uint32_t k4a_device_count = k4a::device::get_installed_count();

    RCLCPP_INFO_STREAM(this->get_logger(), "Found " << k4a_device_count << " sensors");

    if (pSensorSn != "")
    {
      RCLCPP_INFO_STREAM(this->get_logger(), "Searching for sensor with serial number: " << pSensorSn);
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "No serial number provided: picking first sensor");
      RCLCPP_WARN_EXPRESSION(this->get_logger(), k4a_device_count > 1, "Multiple sensors connected! Picking first sensor.");
    }

    for (uint32_t i = 0; i < k4a_device_count; i++)
    {
      k4a::device device;
      try
      {
        device = k4a::device::open(i);
      }
      catch (const std::exception&)
      {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to open K4A device at index " << i);
        continue;
      }

      RCLCPP_INFO_STREAM(this->get_logger(), "K4A[" << i << "] : " << device.get_serialnum());

      // Try to match serial number
      if (pSensorSn!= "")
      {
        if (device.get_serialnum() == pSensorSn)
        {
          k4a_device_ = std::move(device);
          break;
        }
      }
      // Pick the first device
      else if (i == 0)
      {
        k4a_device_ = std::move(device);
        break;
      }
    }

    if (!k4a_device_)
    {
      RCLCPP_FATAL(this->get_logger(), "Failed to open a K4A device. Cannot continue.");
      rclcpp::shutdown();
      return;
    }

    RCLCPP_INFO_STREAM(this->get_logger(), "K4A Serial Number: " << k4a_device_.get_serialnum());

    k4a_hardware_version_t version_info = k4a_device_.get_version();

    RCLCPP_INFO(this->get_logger(), "RGB Version: %d.%d.%d", version_info.rgb.major, version_info.rgb.minor, version_info.rgb.iteration);

    RCLCPP_INFO(this->get_logger(), "Depth Version: %d.%d.%d", version_info.depth.major, version_info.depth.minor,
             version_info.depth.iteration);

    RCLCPP_INFO(this->get_logger(), "Audio Version: %d.%d.%d", version_info.audio.major, version_info.audio.minor,
             version_info.audio.iteration);

    RCLCPP_INFO(this->get_logger(), "Depth Sensor Version: %d.%d.%d", version_info.depth_sensor.major, version_info.depth_sensor.minor,
             version_info.depth_sensor.iteration);
  }


  // TODO: QoS Params
  // qos_.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);
  // qos_.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  // qos_.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
  qos_.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);
  qos_.keep_last(1);
  qos_.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  qos_.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

  std::string topic_prefix = "k4a/";


  // Register our topics
  if (pColorFormat == "jpeg")
  {
    // JPEG images are directly published on 'rgb/image_raw/compressed' so that
    // others can subscribe to 'rgb/image_raw' with compressed_image_transport.
    // This technique is described in:
    // http://wiki.ros.org/compressed_image_transport#Publishing_compressed_images_directly

    // I guess CompressedImage cannot use CameraPublisher. It needs its own separate publishers for the image
    // and the camera_info. https://answers.ros.org/question/385599/how-to-publish-a-compressedimage-in-ros2-foxy/
    rgb_jpeg_publisher_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
        "rgb/image_raw/compressed", qos_);

    rgb_cam_info_jpeg_publisher_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
        "rgb/camera_info", qos_);

    RCLCPP_INFO_STREAM(this->get_logger(),
                       "Advertised on topic: " << rgb_jpeg_publisher_->get_topic_name());
  }
  else if (pColorFormat == "bgra")
  {
    rgb_raw_publisher_ = image_transport::create_camera_publisher(this,
                                                                  topic_prefix + "rgb/image_raw",
                                                                  qos_.get_rmw_qos_profile());
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "Advertised on topic: " << rgb_raw_publisher_.getTopic());
  }

  depth_raw_publisher_ = image_transport::create_camera_publisher(this,
                                                                  topic_prefix + "depth/image_raw",
                                                                  qos_.get_rmw_qos_profile());
  RCLCPP_INFO_STREAM(this->get_logger(),
                     "Advertised on topic: " << depth_raw_publisher_.getTopic());

  depth_rect_publisher_ = image_transport::create_camera_publisher(this,
                                                                   topic_prefix + "depth_to_rgb/image_raw",
                                                                   qos_.get_rmw_qos_profile());
  RCLCPP_INFO_STREAM(this->get_logger(),
                     "Advertised on topic: " << depth_rect_publisher_.getTopic());
  rgb_rect_publisher_ = image_transport::create_camera_publisher(this,
                                                                 topic_prefix + "rgb_to_depth/image_raw",
                                                                 qos_.get_rmw_qos_profile());
  RCLCPP_INFO_STREAM(this->get_logger(),
                     "Advertised on topic: " << rgb_rect_publisher_.getTopic());
  ir_raw_publisher_ = image_transport::create_camera_publisher(this,
                                                               topic_prefix + "ir/image_raw",
                                                               qos_.get_rmw_qos_profile());
  RCLCPP_INFO_STREAM(this->get_logger(),
                     "Advertised on topic: " << ir_raw_publisher_.getTopic());

  imu_orientation_publisher_ = create_publisher<sensor_msgs::msg::Imu>(topic_prefix + "imu",
                                                                       qos_);
  RCLCPP_INFO_STREAM(get_logger(),
                     "Advertised on topic: " << imu_orientation_publisher_->get_topic_name());

  if (this->get_parameter("point_cloud").as_bool() || this->get_parameter("rgb_point_cloud").as_bool()) {
    process_cloud_ = true;
    pointcloud_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(topic_prefix + "points2",
                                                                            qos_);
  }

  diagnostics_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("~/diagnostics", 1);
  diagnostics_timer_ = create_wall_timer(std::chrono::seconds(1), [this] { publishDiagnostics(); });

}

K4AROS2Device::~K4AROS2Device()
{
  running_ = false;
  pending_capture_.close();
  // Capture uses a finite SDK wait. No SDK handle is closed while a worker uses it.
  if (capture_thread_.joinable()) capture_thread_.join();
  if (frame_publisher_thread_.joinable()) frame_publisher_thread_.join();
  if (imu_publisher_thread_.joinable()) imu_publisher_thread_.join();
  stopImu();
  stopCameras();
  if (k4a_playback_handle_) k4a_playback_handle_.close();
}

void K4AROS2Device::workerFailed(const char* worker, const std::exception& error)
{
  RCLCPP_ERROR(get_logger(), "%s failed: %s", worker, error.what());
  failed_ = true;
  running_ = false;
  pending_capture_.close();
  // Keep the executor alive to publish an explicit ERROR health state.
}

k4a_result_t K4AROS2Device::startCameras()
{
  if (running_ || !rclcpp::ok()) return K4A_RESULT_FAILED;
  if (K4AROSDeviceParams::GetDeviceConfig(&device_config_, this) != K4A_RESULT_SUCCEEDED)
    return K4A_RESULT_FAILED;

  depth_enabled_ = get_parameter("depth_enabled").as_bool();
  color_enabled_ = get_parameter("color_enabled").as_bool();
  color_format_ = get_parameter("color_format").as_string();
  rgb_cloud_ = get_parameter("rgb_point_cloud").as_bool();
  cloud_in_depth_ = get_parameter("point_cloud_in_depth_frame").as_bool();
  try
  {
    if (k4a_device_)
    {
      calibration_data_->initialize(k4a_device_, device_config_.depth_mode, device_config_.color_resolution);
      if (driver_color_decode_ && color_enabled_ && color_format_ == "bgra")
      {
        jpeg_decoder_ = std::make_unique<azure_kinect_ros2_driver::MjpegDecoder>();
        device_config_.color_format = K4A_IMAGE_FORMAT_COLOR_MJPG;
      }
      k4a_device_.start_cameras(&device_config_);
      cameras_started_ = true;
    }
    else if (k4a_playback_handle_)
    {
      calibration_data_->initialize(k4a_playback_handle_);
    }
    else return K4A_RESULT_FAILED;

    depth_raw_camerainfo_msg_ = std::make_shared<sensor_msgs::msg::CameraInfo>();
    rgb_raw_camerainfo_msg_ = std::make_shared<sensor_msgs::msg::CameraInfo>();
    calibration_data_->getDepthCameraInfo(depth_raw_camerainfo_msg_);
    calibration_data_->getRgbCameraInfo(rgb_raw_camerainfo_msg_);
    running_ = true;
    frame_publisher_thread_ = std::thread([this] {
      try { framePublisherThread(); }
      catch (const std::exception& e) { workerFailed("Frame worker", e); }
    });
    if (k4a_device_)
      capture_thread_ = std::thread([this] {
        try { captureThread(); }
        catch (const std::exception& e) { workerFailed("Capture worker", e); }
      });
    return K4A_RESULT_SUCCEEDED;
  }
  catch (const std::exception& e)
  {
    workerFailed("Camera startup", e);
    return K4A_RESULT_FAILED;
  }
}

k4a_result_t K4AROS2Device::startImu()
{
  try
  {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    if (!running_) return K4A_RESULT_FAILED;
    if (k4a_device_)
    {
      k4a_device_.start_imu();
      imu_started_ = true;
    }
    imu_publisher_thread_ = std::thread([this] {
      try { imuPublisherThread(); }
      catch (const std::exception& e) { workerFailed("IMU worker", e); }
    });
    return K4A_RESULT_SUCCEEDED;
  }
  catch (const std::exception& e)
  {
    workerFailed("IMU startup", e);
    return K4A_RESULT_FAILED;
  }
}

void K4AROS2Device::stopCameras()
{
  if (k4a_device_ && cameras_started_)
  {
    k4a_device_.stop_cameras();
    cameras_started_ = false;
  }
}

void K4AROS2Device::stopImu()
{
  std::lock_guard<std::mutex> lock(imu_mutex_);
  if (k4a_device_ && imu_started_)
  {
    k4a_device_.stop_imu();
    imu_started_ = false;
  }
}

bool K4AROS2Device::recoverStreams()
{
  recovering_ = true;
  pending_capture_.clear();
  ++stream_generation_;
  // Blocks only IMU reads, not ROS publishing or capture-buffer destruction.
  std::unique_lock<std::mutex> lock(imu_mutex_);
  const bool restart_imu = imu_started_;
  for (int attempt = 0; attempt < recovery_max_attempts_ && running_ && rclcpp::ok(); ++attempt)
  {
    if (imu_started_) { k4a_device_.stop_imu(); imu_started_ = false; }
    stopCameras();
    // Interruptible backoff; never sleep indefinitely during shutdown.
    const auto until = std::chrono::steady_clock::now() +
        std::chrono::milliseconds(static_cast<int64_t>(recovery_backoff_ms_) * (attempt + 1));
    while (running_ && rclcpp::ok() && std::chrono::steady_clock::now() < until)
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    if (!running_ || !rclcpp::ok()) break;
    try
    {
      k4a_device_.start_cameras(&device_config_);
      cameras_started_ = true;
      if (restart_imu) { k4a_device_.start_imu(); imu_started_ = true; }
      ++stream_restarts_;
      recovering_ = false;
      RCLCPP_WARN(get_logger(), "Camera streams restarted; awaiting fresh captures");
      return true;
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(get_logger(), "Stream restart %d/%d failed: %s",
                   attempt + 1, recovery_max_attempts_, e.what());
    }
  }
  recovering_ = false;
  if (running_ && rclcpp::ok())
    workerFailed("Recovery", std::runtime_error("Stream restart budget exhausted"));
  return false;
}

void K4AROS2Device::captureThread()
{
  auto last_success = std::chrono::steady_clock::now();
  const auto healthy_frame_count = static_cast<unsigned int>(get_parameter("fps").as_int());
  unsigned int unstable_restarts = 0;
  unsigned int healthy_captures = 0;
  while (running_ && rclcpp::ok())
  {
    k4a::capture capture;
    try
    {
      if (stream_restart_requested_.exchange(false))
        throw std::runtime_error("IMU requested stream recovery");
      if (!k4a_device_.get_capture(&capture, std::chrono::milliseconds(100)))
      {
        if (std::chrono::steady_clock::now() - last_success <
            std::chrono::milliseconds(capture_timeout_ms_)) continue;
        throw std::runtime_error("No capture within capture_timeout_ms");
      }
      const auto now = std::chrono::steady_clock::now();
      last_success = now;
      last_capture_steady_ns_ = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
      ++captures_received_;
      // Reset the failure budget only after a full second of healthy frames.
      if (healthy_captures < healthy_frame_count) ++healthy_captures;
      if (healthy_captures >= healthy_frame_count)
        unstable_restarts = 0;
      if (pending_capture_.push(CapturePacket{std::move(capture), stream_generation_.load()})) ++captures_replaced_;
    }
    catch (const std::exception& e)
    {
      if (!running_ || !rclcpp::ok()) break;
      ++capture_errors_;
      healthy_captures = 0;
      RCLCPP_ERROR(get_logger(), "Capture stream failed: %s", e.what());
      if (++unstable_restarts > static_cast<unsigned int>(recovery_max_attempts_))
      {
        workerFailed("Capture", std::runtime_error("Repeated stream failures; recovery budget exhausted"));
        break;
      }
      if (!recoverStreams()) break;
      last_success = std::chrono::steady_clock::now();
    }
  }
  pending_capture_.close();
  stopImu();
  stopCameras();
}

void K4AROS2Device::ensureDepthToColor(const k4a::capture& capture)
{
  if (!depth_to_color_ready_)
  {
    calibration_data_->k4a_transformation_.depth_image_to_color_camera(
        capture.get_depth_image(), &calibration_data_->transformed_depth_image_);
    depth_to_color_ready_ = true;
  }
}

void K4AROS2Device::ensureColorToDepth(const k4a::capture& capture)
{
  if (!color_to_depth_ready_)
  {
    calibration_data_->k4a_transformation_.color_image_to_depth_camera(
        capture.get_depth_image(), capture.get_color_image(), &calibration_data_->transformed_rgb_image_);
    color_to_depth_ready_ = true;
  }
}

bool K4AROS2Device::decodeColor(k4a::capture& capture)
{
  auto encoded = capture.get_color_image();
  if (!encoded || encoded.get_format() != K4A_IMAGE_FORMAT_COLOR_MJPG) return true;
  auto decoded = jpeg_decoder_ ? jpeg_decoder_->decode(encoded,
      calibration_data_->getColorWidth(), calibration_data_->getColorHeight()) : k4a::image{};
  if (!decoded)
  {
    ++decode_errors_;
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Dropping malformed color frame; retaining depth/IR");
    capture.set_color_image(k4a::image{});
    return false;
  }
  capture.set_color_image(decoded);
  return true;
}

k4a_result_t K4AROS2Device::getDepthFrame(const k4a::capture& capture, std::shared_ptr<sensor_msgs::msg::Image>& depth_image,
                                         bool rectified = false)
{
  k4a::image k4a_depth_frame = capture.get_depth_image();

  if (!k4a_depth_frame)
  {
    RCLCPP_ERROR(this->get_logger(), "Cannot render depth frame: no frame");
    return K4A_RESULT_FAILED;
  }

  if (rectified)
  {
    ensureDepthToColor(capture);

    return renderDepthToROS(depth_image, calibration_data_->transformed_depth_image_);
  }

  return renderDepthToROS(depth_image, k4a_depth_frame);
}

k4a_result_t K4AROS2Device::renderDepthToROS(std::shared_ptr<sensor_msgs::msg::Image>& depth_image, k4a::image& k4a_depth_frame)
{
  cv::Mat depth_frame_buffer_mat(k4a_depth_frame.get_height_pixels(), k4a_depth_frame.get_width_pixels(), CV_16UC1,
                                 k4a_depth_frame.get_buffer(), k4a_depth_frame.get_stride_bytes());
  depth_image = std::make_shared<sensor_msgs::msg::Image>();
  depth_image->height = k4a_depth_frame.get_height_pixels();
  depth_image->width = k4a_depth_frame.get_width_pixels();
  depth_image->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  depth_image->is_bigendian = false;
  depth_image->step = depth_image->width * sizeof(float);
  depth_image->data.resize(static_cast<size_t>(depth_image->step) * depth_image->height);
  // Convert directly into ROS-owned storage, without an intermediate float image
  // and cv_bridge copy. Preserve metre-valued 32FC1 output.
  cv::Mat output(depth_image->height, depth_image->width, CV_32FC1, depth_image->data.data(), depth_image->step);
  depth_frame_buffer_mat.convertTo(output, CV_32FC1, 0.001);

  return K4A_RESULT_SUCCEEDED;
}

k4a_result_t K4AROS2Device::getIrFrame(const k4a::capture& capture, std::shared_ptr<sensor_msgs::msg::Image>& ir_image)
{
  k4a::image k4a_ir_frame = capture.get_ir_image();

  if (!k4a_ir_frame)
  {
    RCLCPP_ERROR(this->get_logger(), "Cannot render IR frame: no frame");
    return K4A_RESULT_FAILED;
  }

  return renderIrToROS(ir_image, k4a_ir_frame);
}

k4a_result_t K4AROS2Device::renderIrToROS(std::shared_ptr<sensor_msgs::msg::Image>& ir_image, k4a::image& k4a_ir_frame)
{
  cv::Mat ir_buffer_mat(k4a_ir_frame.get_height_pixels(), k4a_ir_frame.get_width_pixels(), CV_16UC1,
                        k4a_ir_frame.get_buffer(), k4a_ir_frame.get_stride_bytes());

  // Rescale the image to mono8 for visualization and usage for visual(-inertial) odometry.
  if (this->get_parameter("rescale_ir_to_mono8").as_bool())
  {
    cv::Mat new_image(k4a_ir_frame.get_height_pixels(), k4a_ir_frame.get_width_pixels(), CV_8UC1);
    // Use a scaling factor to re-scale the image. If using the illuminators, a value of 1 is appropriate.
    // If using PASSIVE_IR, then a value of 10 is more appropriate; k4aviewer does a similar conversion.
    ir_buffer_mat.convertTo(new_image, CV_8UC1, this->get_parameter("ir_mono8_scaling_factor").as_double());
    ir_image = cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::MONO8, new_image).toImageMsg();
  }
  else
  {
    ir_image = cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::MONO16, ir_buffer_mat).toImageMsg();
  }

  return K4A_RESULT_SUCCEEDED;
}

k4a_result_t K4AROS2Device::getJpegRgbFrame(const k4a::capture& capture, std::shared_ptr<sensor_msgs::msg::CompressedImage> jpeg_image)
{
  k4a::image k4a_jpeg_frame = capture.get_color_image();

  if (!k4a_jpeg_frame)
  {
    RCLCPP_ERROR(this->get_logger(), "Cannot render Jpeg frame: no frame");
    return K4A_RESULT_FAILED;
  }

  const uint8_t* jpeg_frame_buffer = k4a_jpeg_frame.get_buffer();
  jpeg_image->format = "bgra8; jpeg compressed bgr8";
  jpeg_image->data.assign(jpeg_frame_buffer, jpeg_frame_buffer + k4a_jpeg_frame.get_size());
  return K4A_RESULT_SUCCEEDED;
}

k4a_result_t K4AROS2Device::getRbgFrame(const k4a::capture& capture, std::shared_ptr<sensor_msgs::msg::Image>& rgb_image,
                                       bool rectified = false)
{
  k4a::image k4a_bgra_frame = capture.get_color_image();

  if (!k4a_bgra_frame)
  {
    RCLCPP_ERROR(this->get_logger(), "Cannot render BGRA frame: no frame");
    return K4A_RESULT_FAILED;
  }

  size_t color_image_size =
      static_cast<size_t>(k4a_bgra_frame.get_width_pixels() * k4a_bgra_frame.get_height_pixels()) * sizeof(BgraPixel);

  if (k4a_bgra_frame.get_size() != color_image_size)
  {
    RCLCPP_WARN(this->get_logger(), "Invalid k4a_bgra_frame returned from K4A");
    return K4A_RESULT_FAILED;
  }

  if (rectified)
  {
    k4a::image k4a_depth_frame = capture.get_depth_image();

    ensureColorToDepth(capture);


    return renderBGRA32ToROS(rgb_image, calibration_data_->transformed_rgb_image_);
  }

  return renderBGRA32ToROS(rgb_image, k4a_bgra_frame);
}

// Helper function that renders any BGRA K4A frame to a ROS ImagePtr. Useful for rendering intermediary frames
// during debugging of image processing functions
k4a_result_t K4AROS2Device::renderBGRA32ToROS(std::shared_ptr<sensor_msgs::msg::Image>& rgb_image, k4a::image& k4a_bgra_frame)
{
  cv::Mat rgb_buffer_mat(k4a_bgra_frame.get_height_pixels(), k4a_bgra_frame.get_width_pixels(), CV_8UC4,
                         k4a_bgra_frame.get_buffer(), k4a_bgra_frame.get_stride_bytes());

  rgb_image = cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::BGRA8, rgb_buffer_mat).toImageMsg();

  return K4A_RESULT_SUCCEEDED;
}

k4a_result_t K4AROS2Device::getRgbPointCloudInDepthFrame(const k4a::capture& capture,
                                                         std::shared_ptr<sensor_msgs::msg::PointCloud2> point_cloud)
{
  const k4a::image k4a_depth_frame = capture.get_depth_image();
  if (!k4a_depth_frame)
  {
    RCLCPP_ERROR(this->get_logger(), "Cannot render RGB point cloud: no depth frame");
    return K4A_RESULT_FAILED;
  }

  const k4a::image k4a_bgra_frame = capture.get_color_image();
  if (!k4a_bgra_frame)
  {
    RCLCPP_ERROR(this->get_logger(), "Cannot render RGB point cloud: no BGRA frame");
    return K4A_RESULT_FAILED;
  }

  // Transform color image into the depth camera frame:
  ensureColorToDepth(capture);

  // Tranform depth image to point cloud
  calibration_data_->k4a_transformation_.depth_image_to_point_cloud(k4a_depth_frame, K4A_CALIBRATION_TYPE_DEPTH,
                                                                   &calibration_data_->point_cloud_image_);

  point_cloud->header.frame_id = calibration_data_->tf_prefix_ + calibration_data_->depth_camera_frame_;
  point_cloud->header.stamp = timestampToROS(k4a_depth_frame.get_device_timestamp());
  this->printTimestampDebugMessage("RGB point cloud", point_cloud->header.stamp);

  return fillColorPointCloud(calibration_data_->point_cloud_image_, calibration_data_->transformed_rgb_image_,
                             point_cloud);
}

k4a_result_t K4AROS2Device::getRgbPointCloudInRgbFrame(const k4a::capture& capture,
                                                       std::shared_ptr<sensor_msgs::msg::PointCloud2>  point_cloud)
{
  k4a::image k4a_depth_frame = capture.get_depth_image();
  if (!k4a_depth_frame)
  {
    RCLCPP_ERROR(this->get_logger(), "Cannot render RGB point cloud: no depth frame");
    return K4A_RESULT_FAILED;
  }

  k4a::image k4a_bgra_frame = capture.get_color_image();
  if (!k4a_bgra_frame)
  {
    RCLCPP_ERROR(this->get_logger(), "Cannot render RGB point cloud: no BGRA frame");
    return K4A_RESULT_FAILED;
  }

  // transform depth image into color camera geometry
  ensureDepthToColor(capture);

  // Tranform depth image to point cloud (note that this is now from the perspective of the color camera)
  calibration_data_->k4a_transformation_.depth_image_to_point_cloud(
      calibration_data_->transformed_depth_image_, K4A_CALIBRATION_TYPE_COLOR, &calibration_data_->point_cloud_image_);

  point_cloud->header.frame_id = calibration_data_->tf_prefix_ + calibration_data_->rgb_camera_frame_;
  point_cloud->header.stamp = timestampToROS(k4a_bgra_frame.get_device_timestamp());
  this->printTimestampDebugMessage("RGB point cloud", point_cloud->header.stamp);

  return fillColorPointCloud(calibration_data_->point_cloud_image_, k4a_bgra_frame, point_cloud);
}

k4a_result_t K4AROS2Device::getPointCloud(const k4a::capture& capture, std::shared_ptr<sensor_msgs::msg::PointCloud2>  point_cloud)
{
  k4a::image k4a_depth_frame = capture.get_depth_image();

  if (!k4a_depth_frame)
  {
    RCLCPP_ERROR(this->get_logger(), "Cannot render point cloud: no depth frame");
    return K4A_RESULT_FAILED;
  }

  point_cloud->header.frame_id = calibration_data_->tf_prefix_ + calibration_data_->depth_camera_frame_;
  point_cloud->header.stamp = timestampToROS(k4a_depth_frame.get_device_timestamp());
  this->printTimestampDebugMessage("Point cloud", point_cloud->header.stamp);

  // Tranform depth image to point cloud
  calibration_data_->k4a_transformation_.depth_image_to_point_cloud(k4a_depth_frame, K4A_CALIBRATION_TYPE_DEPTH,
                                                                   &calibration_data_->point_cloud_image_);

  return fillPointCloud(calibration_data_->point_cloud_image_, point_cloud);
}

k4a_result_t K4AROS2Device::fillColorPointCloud(const k4a::image& pointcloud_image, const k4a::image& color_image,
                                                std::shared_ptr<sensor_msgs::msg::PointCloud2>&  point_cloud)
{
  point_cloud->height = pointcloud_image.get_height_pixels();
  point_cloud->width = pointcloud_image.get_width_pixels();
  point_cloud->is_dense = false;
  point_cloud->is_bigendian = false;

  const size_t point_count = pointcloud_image.get_height_pixels() * pointcloud_image.get_width_pixels();
  const size_t pixel_count = color_image.get_size() / sizeof(BgraPixel);
  if (point_count != pixel_count)
  {
    RCLCPP_WARN(this->get_logger(), "Color and depth image sizes do not match!");
    return K4A_RESULT_FAILED;
  }

  sensor_msgs::PointCloud2Modifier pcd_modifier(*point_cloud);
  pcd_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  pcd_modifier.resize(point_count);

  sensor_msgs::PointCloud2Iterator<float> iter_x(*point_cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*point_cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(*point_cloud, "z");

  sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(*point_cloud, "r");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(*point_cloud, "g");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(*point_cloud, "b");

  const int16_t* point_cloud_buffer = reinterpret_cast<const int16_t*>(pointcloud_image.get_buffer());
  const uint8_t* color_buffer = color_image.get_buffer();

  for (size_t i = 0; i < point_count; i++, ++iter_x, ++iter_y, ++iter_z, ++iter_r, ++iter_g, ++iter_b)
  {
    // Z in image frame:
    float z = static_cast<float>(point_cloud_buffer[3 * i + 2]);
    // Alpha value:
    uint8_t a = color_buffer[4 * i + 3];
    if (z <= 0.0f || a == 0)
    {
      *iter_x = *iter_y = *iter_z = std::numeric_limits<float>::quiet_NaN();
      *iter_r = *iter_g = *iter_b = 0;
    }
    else
    {
      constexpr float kMillimeterToMeter = 1.0 / 1000.0f;
      *iter_x = kMillimeterToMeter * static_cast<float>(point_cloud_buffer[3 * i + 0]);
      *iter_y = kMillimeterToMeter * static_cast<float>(point_cloud_buffer[3 * i + 1]);
      *iter_z = kMillimeterToMeter * z;

      *iter_r = color_buffer[4 * i + 2];
      *iter_g = color_buffer[4 * i + 1];
      *iter_b = color_buffer[4 * i + 0];
    }
  }

  return K4A_RESULT_SUCCEEDED;
}

k4a_result_t K4AROS2Device::fillPointCloud(const k4a::image& pointcloud_image,
                                           std::shared_ptr<sensor_msgs::msg::PointCloud2> point_cloud)
{
  point_cloud->height = pointcloud_image.get_height_pixels();
  point_cloud->width = pointcloud_image.get_width_pixels();
  point_cloud->is_dense = false;
  point_cloud->is_bigendian = false;

  const size_t point_count = pointcloud_image.get_height_pixels() * pointcloud_image.get_width_pixels();

  sensor_msgs::PointCloud2Modifier pcd_modifier(*point_cloud);
  pcd_modifier.setPointCloud2FieldsByString(1, "xyz");
  pcd_modifier.resize(point_count);

  sensor_msgs::PointCloud2Iterator<float> iter_x(*point_cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*point_cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(*point_cloud, "z");

  const int16_t* point_cloud_buffer = reinterpret_cast<const int16_t*>(pointcloud_image.get_buffer());

  for (size_t i = 0; i < point_count; i++, ++iter_x, ++iter_y, ++iter_z)
  {
    float z = static_cast<float>(point_cloud_buffer[3 * i + 2]);

    if (z <= 0.0f)
    {
      *iter_x = *iter_y = *iter_z = std::numeric_limits<float>::quiet_NaN();
    }
    else
    {
      constexpr float kMillimeterToMeter = 1.0 / 1000.0f;
      *iter_x = kMillimeterToMeter * static_cast<float>(point_cloud_buffer[3 * i + 0]);
      *iter_y = kMillimeterToMeter * static_cast<float>(point_cloud_buffer[3 * i + 1]);
      *iter_z = kMillimeterToMeter * z;
    }
  }

  return K4A_RESULT_SUCCEEDED;
}

k4a_result_t K4AROS2Device::getImuFrame(const k4a_imu_sample_t& sample, std::shared_ptr<sensor_msgs::msg::Imu> imu_msg)
{
  imu_msg->header.frame_id = calibration_data_->tf_prefix_ + calibration_data_->imu_frame_;
  imu_msg->header.stamp = timestampToROS(sample.acc_timestamp_usec);
  this->printTimestampDebugMessage("IMU", imu_msg->header.stamp);

  // The correct convention in ROS is to publish the raw sensor data, in the
  // sensor coordinate frame. Do that here.
  imu_msg->angular_velocity.x = sample.gyro_sample.xyz.x;
  imu_msg->angular_velocity.y = sample.gyro_sample.xyz.y;
  imu_msg->angular_velocity.z = sample.gyro_sample.xyz.z;

  imu_msg->linear_acceleration.x = sample.acc_sample.xyz.x;
  imu_msg->linear_acceleration.y = sample.acc_sample.xyz.y;
  imu_msg->linear_acceleration.z = sample.acc_sample.xyz.z;

  // Disable the orientation component of the IMU message since it's invalid
  imu_msg->orientation_covariance[0] = -1.0;

  return K4A_RESULT_SUCCEEDED;
}


void K4AROS2Device::framePublisherThread()
{
  rclcpp::WallRate playback_rate(get_parameter("fps").as_int());
  while (running_ && rclcpp::ok())
  {
    k4a::capture capture;
    uint64_t generation = 0;
    if (k4a_device_)
    {
      CapturePacket packet;
      if (!pending_capture_.pop(packet, std::chrono::milliseconds(100))) continue;
      capture = std::move(packet.capture);
      generation = packet.generation;
      if (recovering_) continue;
    }
    else
    {
      std::lock_guard<std::mutex> guard(k4a_playback_handle_mutex_);
      if (!k4a_playback_handle_.get_next_capture(&capture))
      {
        if (!get_parameter("recording_loop_enabled").as_bool())
        {
          RCLCPP_INFO(get_logger(), "Recording reached end of file");
          running_ = false;
          rclcpp::shutdown();
          return;
        }
        k4a_playback_handle_.seek_timestamp(std::chrono::microseconds(0), K4A_PLAYBACK_SEEK_BEGIN);
        if (!k4a_playback_handle_.get_next_capture(&capture))
          throw std::runtime_error("Recording contains no captures");
        imu_stream_end_of_file_ = false;
        last_imu_time_usec_ = 0;
      }
      last_capture_time_usec_ = getCaptureTimestamp(capture).count();
    }

    const auto started = std::chrono::steady_clock::now();
    try { processCapture(std::move(capture), generation); }
    catch (const std::exception& e)
    {
      ++processing_errors_;
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "Frame processing failed: %s", e.what());
    }
    last_processing_us_ = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now() - started).count();
    // Live capture is paced by the device, never by an additional ROS rate sleep.
    if (k4a_playback_handle_) playback_rate.sleep();
  }
}

void K4AROS2Device::processCapture(k4a::capture capture, uint64_t generation)
{
  depth_to_color_ready_ = false;
  color_to_depth_ready_ = false;
  auto depth = capture.get_depth_image();
  auto color = capture.get_color_image();
  auto ir = capture.get_ir_image();
  auto reference = ir ? ir : color;
  if (!reference) return;
  // On Linux the SDK system timestamp and steady_clock both use CLOCK_MONOTONIC.
  const auto fresh = [&] {
    if (!k4a_device_) return true;
    if (!running_ || recovering_ || stream_generation_.load() != generation) return false;
    const auto age = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch() - reference.get_system_timestamp()).count();
    last_frame_age_us_ = age;
    return max_capture_age_ms_ == 0 || age <= static_cast<int64_t>(max_capture_age_ms_) * 1000;
  };
  if (!fresh()) { ++stale_captures_; return; }
  if (k4a_device_)
    updateTimestampOffset(reference.get_device_timestamp(), reference.get_system_timestamp());

  const auto check = [](k4a_result_t result) {
    if (result != K4A_RESULT_SUCCEEDED) throw std::runtime_error("Image/point cloud conversion failed");
  };
  const auto publish_image = [&](image_transport::CameraPublisher& publisher,
                                 std::shared_ptr<sensor_msgs::msg::Image>& image,
                                 const k4a::image& source, bool color_geometry) {
    if (!fresh()) { ++stale_captures_; return; }
    // Cached calibration is immutable; each publication gets its own header.
    auto info = std::make_shared<sensor_msgs::msg::CameraInfo>(
        color_geometry ? *rgb_raw_camerainfo_msg_ : *depth_raw_camerainfo_msg_);
    image->header = info->header;
    image->header.stamp = timestampToROS(source.get_device_timestamp());
    info->header.stamp = image->header.stamp;
    publisher.publish(image, info);
  };

  // Publish geometry first; optional color processing must not delay native depth.
  if (depth_enabled_ && depth && depth_raw_publisher_.getNumSubscribers() > 0)
  {
    std::shared_ptr<sensor_msgs::msg::Image> image;
    check(getDepthFrame(capture, image));
    publish_image(depth_raw_publisher_, image, depth, false);
  }
  const bool cloud_requested = process_cloud_ && pointcloud_publisher_->get_subscription_count() > 0;
  if (depth && cloud_requested && !rgb_cloud_ && fresh())
  {
    auto cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
    check(getPointCloud(capture, cloud));
    if (fresh()) pointcloud_publisher_->publish(*cloud);
    else ++stale_captures_;
  }
  if (depth_enabled_ && ir && ir_raw_publisher_.getNumSubscribers() > 0 && fresh())
  {
    std::shared_ptr<sensor_msgs::msg::Image> image;
    check(getIrFrame(capture, image));
    publish_image(ir_raw_publisher_, image, ir, false);
  }
  if (depth && color_enabled_ && depth_rect_publisher_.getNumSubscribers() > 0 && fresh())
  {
    std::shared_ptr<sensor_msgs::msg::Image> image;
    check(getDepthFrame(capture, image, true));
    publish_image(depth_rect_publisher_, image, depth, true);
  }

  if (color_enabled_ && color && fresh())
  {
    if (color_format_ == "jpeg")
    {
      if (rgb_jpeg_publisher_->get_subscription_count() > 0 || rgb_cam_info_jpeg_publisher_->get_subscription_count() > 0)
      {
        auto info = std::make_shared<sensor_msgs::msg::CameraInfo>(*rgb_raw_camerainfo_msg_);
        info->header.stamp = timestampToROS(color.get_device_timestamp());
        if (rgb_jpeg_publisher_->get_subscription_count() > 0)
        {
          auto image = std::make_shared<sensor_msgs::msg::CompressedImage>();
          check(getJpegRgbFrame(capture, image));
          image->header = info->header;
          if (fresh()) rgb_jpeg_publisher_->publish(*image);
          else ++stale_captures_;
        }
        if (fresh()) rgb_cam_info_jpeg_publisher_->publish(*info);
      }
    }
    else if (rgb_raw_publisher_.getNumSubscribers() > 0 ||
             (depth && rgb_rect_publisher_.getNumSubscribers() > 0) || (cloud_requested && rgb_cloud_))
    {
      if (decodeColor(capture))
      {
        color = capture.get_color_image();
        if (rgb_raw_publisher_.getNumSubscribers() > 0 && fresh())
        {
          std::shared_ptr<sensor_msgs::msg::Image> image;
          check(getRbgFrame(capture, image));
          publish_image(rgb_raw_publisher_, image, color, true);
        }
        if (depth && rgb_rect_publisher_.getNumSubscribers() > 0 && fresh())
        {
          std::shared_ptr<sensor_msgs::msg::Image> image;
          check(getRbgFrame(capture, image, true));
          publish_image(rgb_rect_publisher_, image, color, false);
        }
        if (depth && cloud_requested && rgb_cloud_ && fresh())
        {
          auto cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
          check(cloud_in_depth_ ? getRgbPointCloudInDepthFrame(capture, cloud) : getRgbPointCloudInRgbFrame(capture, cloud));
          if (fresh()) pointcloud_publisher_->publish(*cloud);
          else ++stale_captures_;
        }
      }
    }
  }
  last_processed_steady_ns_ = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
}

k4a_imu_sample_t K4AROS2Device::computeMeanIMUSample(const std::vector<k4a_imu_sample_t>& samples)
{
  // Compute mean sample
  // Using double-precision version of imu sample struct to avoid overflow
  k4a_imu_accumulator_t mean;
  for (auto imu_sample : samples)
  {
    mean += imu_sample;
  }
  float num_samples = samples.size();
  mean /= num_samples;

  // Convert to floating point
  k4a_imu_sample_t mean_float;
  mean.to_float(mean_float);
  // Use most timestamp of most recent sample
  mean_float.acc_timestamp_usec = samples.back().acc_timestamp_usec;
  mean_float.gyro_timestamp_usec = samples.back().gyro_timestamp_usec;

  return mean_float;
}


void K4AROS2Device::imuPublisherThread()
{
  rclcpp::WallRate loop_rate(300);
  const auto target_count = static_cast<size_t>(IMU_MAX_RATE / get_parameter("imu_rate_target").as_int());
  std::vector<k4a_imu_sample_t> samples;
  samples.reserve(target_count);
  uint64_t generation = stream_generation_.load();
  while (running_ && rclcpp::ok())
  {
    if (recovering_ || stream_restart_requested_)
    {
      samples.clear();
      loop_rate.sleep();
      continue;
    }
    // Bound work per iteration so shutdown is checked even under sustained load.
    for (unsigned int i = 0; i < 128 && running_ && rclcpp::ok(); ++i)
    {
      k4a_imu_sample_t sample;
      bool read = false;
      if (k4a_device_)
      {
        std::lock_guard<std::mutex> lock(imu_mutex_);
        if (recovering_ || !imu_started_) break;
        if (generation != stream_generation_.load())
        {
          samples.clear();
          generation = stream_generation_.load();
        }
        try { read = k4a_device_.get_imu_sample(&sample, std::chrono::milliseconds(0)); }
        catch (const k4a::error& e)
        {
          RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "IMU read failed: %s", e.what());
          stream_restart_requested_ = true;
          samples.clear();
          break;
        }
      }
      else
      {
        std::lock_guard<std::mutex> lock(k4a_playback_handle_mutex_);
        if (last_imu_time_usec_ > static_cast<uint64_t>(last_capture_time_usec_.load()) || imu_stream_end_of_file_) break;
        read = k4a_playback_handle_.get_next_imu_sample(&sample);
        if (!read) imu_stream_end_of_file_ = true;
        else last_imu_time_usec_ = sample.acc_timestamp_usec;
      }
      if (!read) break;
      // Always drain the SDK queue, but avoid constructing unused ROS messages.
      if (imu_orientation_publisher_->get_subscription_count() == 0)
      {
        samples.clear();
        continue;
      }
      samples.push_back(sample);
      if (samples.size() < target_count) continue;
      auto msg = std::make_shared<sensor_msgs::msg::Imu>();
      const auto result = getImuFrame(target_count > 1 ? computeMeanIMUSample(samples) : sample, msg);
      samples.clear();
      if (result != K4A_RESULT_SUCCEEDED) throw std::runtime_error("IMU conversion failed");
      if (!k4a_device_ || (!recovering_ && generation == stream_generation_.load()))
        imu_orientation_publisher_->publish(*msg);
    }
    loop_rate.sleep();
  }
}

void K4AROS2Device::publishDiagnostics()
{
  using Status = diagnostic_msgs::msg::DiagnosticStatus;
  diagnostic_msgs::msg::DiagnosticArray msg;
  msg.header.stamp = now();
  Status status;
  status.name = std::string(get_fully_qualified_name()) + "/capture";
  status.hardware_id = "azure_kinect";
  const int64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  const auto capture_ns = last_capture_steady_ns_.load();
  const auto processed_ns = last_processed_steady_ns_.load();
  const auto decodes = decode_errors_.load();
  const auto processing = processing_errors_.load();
  const auto stale = stale_captures_.load();
  status.level = Status::OK;
  status.message = "Capture and processing active";
  if (failed_ || !running_)
  {
    status.level = Status::ERROR;
    status.message = "Sensor stopped; restart node after checking device/USB";
  }
  else if (recovering_)
  {
    status.level = Status::WARN;
    status.message = "Recovering camera streams; sensor data unavailable";
  }
  else if (k4a_device_ && (capture_ns == 0 || processed_ns == 0 ||
           now_ns - capture_ns > static_cast<int64_t>(capture_timeout_ms_) * 1000000 ||
           now_ns - processed_ns > static_cast<int64_t>(capture_timeout_ms_) * 1000000))
  {
    status.level = Status::ERROR;
    status.message = "No recent capture or completed processing cycle";
  }
  else if (decodes != previous_decode_errors_ || processing != previous_processing_errors_ ||
           stale != previous_stale_captures_)
  {
    status.level = Status::WARN;
    status.message = "Color decode, conversion, or stale-output drops in last interval";
  }
  previous_decode_errors_ = decodes;
  previous_processing_errors_ = processing;
  previous_stale_captures_ = stale;
  const auto add = [&](const char* key, int64_t value) {
    diagnostic_msgs::msg::KeyValue entry;
    entry.key = key;
    entry.value = std::to_string(value);
    status.values.push_back(std::move(entry));
  };
  add("captures_received", captures_received_.load());
  add("pending_captures_replaced", captures_replaced_.load());
  add("stale_output_drops", stale);
  add("capture_errors", capture_errors_.load());
  add("color_decode_errors", decodes);
  add("processing_errors", processing);
  add("stream_restarts", stream_restarts_.load());
  add("last_processing_us", last_processing_us_.load());
  add("last_checked_frame_age_us", last_frame_age_us_.load());
  add("ms_since_last_capture", capture_ns ? (now_ns - capture_ns) / 1000000 : -1);
  add("ms_since_last_processed_capture", processed_ns ? (now_ns - processed_ns) / 1000000 : -1);
  msg.status.push_back(std::move(status));
  diagnostics_publisher_->publish(msg);
}

std::chrono::microseconds K4AROS2Device::getCaptureTimestamp(const k4a::capture& capture)
{
  // Captures don't actually have timestamps, images do, so we have to look at all the images
  // associated with the capture.  We just return the first one we get back.
  //
  // We check the IR capture instead of the depth capture because if the depth camera is started
  // in passive IR mode, it only has an IR image (i.e. no depth image), but there is no mode
  // where a capture will have a depth image but not an IR image.
  //
  const auto irImage = capture.get_ir_image();
  if (irImage != nullptr)
  {
    return irImage.get_device_timestamp();
  }

  const auto colorImage = capture.get_color_image();
  if (colorImage != nullptr)
  {
    return colorImage.get_device_timestamp();
  }

  return std::chrono::microseconds::zero();
}

// Converts a k4a *device* timestamp to a ros::Time object
rclcpp::Time K4AROS2Device::timestampToROS(const std::chrono::microseconds& k4a_timestamp_us)
{
  std::lock_guard<std::mutex> lock(timestamp_mutex_);
  // This will give INCORRECT timestamps until the first image.
  if (device_to_realtime_offset_.count() == 0)
  {
    initializeTimestampOffset(k4a_timestamp_us);
  }

  std::chrono::nanoseconds timestamp_in_realtime = k4a_timestamp_us + device_to_realtime_offset_;
  // Set as ROS_TIME clock
  rclcpp::Time ros_time(timestamp_in_realtime.count(), RCL_ROS_TIME);

  return ros_time;
}

// Converts a k4a_imu_sample_t timestamp to a ros::Time object
rclcpp::Time K4AROS2Device::timestampToROS(const uint64_t& k4a_timestamp_us)
{
  return timestampToROS(std::chrono::microseconds(k4a_timestamp_us));
}

void K4AROS2Device::initializeTimestampOffset(const std::chrono::microseconds& k4a_device_timestamp_us)
{
  // We have no better guess than "now".
  std::chrono::nanoseconds realtime_clock = std::chrono::system_clock::now().time_since_epoch();

  device_to_realtime_offset_ = realtime_clock - k4a_device_timestamp_us;

  RCLCPP_WARN_STREAM(this->get_logger(), "Initializing the device to realtime offset based on wall clock: "
                  << device_to_realtime_offset_.count() << " ns");
}

void K4AROS2Device::updateTimestampOffset(const std::chrono::microseconds& k4a_device_timestamp_us,
                                         const std::chrono::nanoseconds& k4a_system_timestamp_ns)
{
  std::lock_guard<std::mutex> lock(timestamp_mutex_);
  // System timestamp is on monotonic system clock.
  // Device time is on AKDK hardware clock.
  // We want to continuously estimate diff between realtime and AKDK hardware clock as low-pass offset.
  // This consists of two parts: device to monotonic, and monotonic to realtime.

  // First figure out realtime to monotonic offset. This will change to keep updating it.
  std::chrono::nanoseconds realtime_clock = std::chrono::system_clock::now().time_since_epoch();
  std::chrono::nanoseconds monotonic_clock = std::chrono::steady_clock::now().time_since_epoch();

  std::chrono::nanoseconds monotonic_to_realtime = realtime_clock - monotonic_clock;

  // Next figure out the other part (combined).
  std::chrono::nanoseconds device_to_realtime =
      k4a_system_timestamp_ns - k4a_device_timestamp_us + monotonic_to_realtime;
  // If we're over a second off, just snap into place.
  const auto offset_error = device_to_realtime_offset_- device_to_realtime;
  if (device_to_realtime_offset_.count() == 0 ||
      std::abs((device_to_realtime_offset_ - device_to_realtime).count()) > 1e7) // ZK - CHANGED THIS FROM 1e7
  {
    // RCLCPP_WARN_STREAM(this->get_logger(), "Initializing or re-initializing the device to realtime offset: "
    //   << device_to_realtime.count() << " ns");

    RCLCPP_WARN(this->get_logger(),"timestamp offset reset: "
              "old=%ld ns, new=%ld ns error = %.3f ms", 
              static_cast<long>(device_to_realtime_offset_.count()),
              static_cast<long>(device_to_realtime.count()),
              static_cast<double>(offset_error.count())/1e6);
  
    
    device_to_realtime_offset_ = device_to_realtime;
  }
  else
  {
    // Low-pass filter!
    constexpr double alpha = 0.10;
    device_to_realtime_offset_ = device_to_realtime_offset_ +
                                 std::chrono::nanoseconds(static_cast<int64_t>(
                                     std::floor(alpha * (device_to_realtime - device_to_realtime_offset_).count())));
  }
}




void K4AROS2Device::printTimestampDebugMessage(const std::string& name, const rclcpp::Time& timestamp)
{
  if (!rcutils_logging_logger_is_enabled_for(get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG)) return;
  std::lock_guard<std::mutex> lock(debug_mutex_);

  rclcpp::Time now(this->now(), RCL_ROS_TIME);
  rclcpp::Duration lag = now - timestamp;

  auto it = map_min_max_.find(name);
  if (it == map_min_max_.end())
  {
    map_min_max_.insert(std::make_pair(name, std::make_pair(lag, lag)));
    it = map_min_max_.find(name);
  }
  else
  {
    auto& min_lag = it->second.first;
    auto& max_lag = it->second.second;
    if (lag < min_lag)
    {
      min_lag = lag;
    }
    if (lag > max_lag)
    {
      max_lag = lag;
    }
  }

  RCLCPP_DEBUG_STREAM(this->get_logger(), name << " timestamp lags node Time::now() by\n"
                        << std::setw(23) << lag.seconds() * 1000.0 << " ms. "
                        << "The lag ranges from " << it->second.first.seconds() * 1000.0 << "ms"
                        << " to " << it->second.second.seconds() * 1000.0 << "ms.");
}
