#include <algorithm>
#include <chrono>
#include <functional>
#include <string>
#include <vector>

#include <camera_info_manager/camera_info_manager.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

class OpenCVUsbCam : public rclcpp::Node
{
public:
  OpenCVUsbCam() : Node("opencv_usb_cam")
  {
    this->declare_parameter("video_device", "/dev/video0");
    this->declare_parameter("image_width", 640);
    this->declare_parameter("image_height", 480);
    this->declare_parameter("framerate", 30);
    this->declare_parameter("camera_frame_id", "camera");
    this->declare_parameter("camera_info_url", "");

    this->declare_parameter("enable_hsv_filter", false);
    this->declare_parameter("publish_hsv_debug_topics", true);
    this->declare_parameter("h_min", 0);
    this->declare_parameter("h_max", 179);
    this->declare_parameter("s_min", 0);
    this->declare_parameter("s_max", 255);
    this->declare_parameter("v_min", 0);
    this->declare_parameter("v_max", 255);

    std::string video_device = this->get_parameter("video_device").as_string();
    int width = static_cast<int>(this->get_parameter("image_width").as_int());
    int height = static_cast<int>(this->get_parameter("image_height").as_int());
    int framerate = static_cast<int>(this->get_parameter("framerate").as_int());
    camera_frame_id_ = this->get_parameter("camera_frame_id").as_string();
    std::string camera_info_url = this->get_parameter("camera_info_url").as_string();

    enable_hsv_filter_ = this->get_parameter("enable_hsv_filter").as_bool();
    publish_hsv_debug_topics_ = this->get_parameter("publish_hsv_debug_topics").as_bool();
    h_min_ = clamp_h(static_cast<int>(this->get_parameter("h_min").as_int()));
    h_max_ = clamp_h(static_cast<int>(this->get_parameter("h_max").as_int()));
    s_min_ = clamp_sv(static_cast<int>(this->get_parameter("s_min").as_int()));
    s_max_ = clamp_sv(static_cast<int>(this->get_parameter("s_max").as_int()));
    v_min_ = clamp_sv(static_cast<int>(this->get_parameter("v_min").as_int()));
    v_max_ = clamp_sv(static_cast<int>(this->get_parameter("v_max").as_int()));

    cap_.open(video_device, cv::CAP_V4L2);
    if (!cap_.isOpened()) {
      RCLCPP_FATAL(this->get_logger(), "Failed to open camera device %s", video_device.c_str());
      rclcpp::shutdown();
      return;
    }
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, width);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height);
    cap_.set(cv::CAP_PROP_FPS, framerate);
    cap_.set(cv::CAP_PROP_BRIGHTNESS, 0);

    cim_ = std::make_shared<camera_info_manager::CameraInfoManager>(
      this, camera_frame_id_, camera_info_url);

    if (!camera_info_url.empty() && !cim_->loadCameraInfo(camera_info_url)) {
      RCLCPP_WARN(this->get_logger(), "Failed to load camera info from %s", camera_info_url.c_str());
    }

    std::string image_topic = camera_frame_id_ + "/image_raw";
    std::string info_topic = camera_frame_id_ + "/camera_info";
    std::string hsv_filtered_topic = camera_frame_id_ + "/image_hsv_filtered";
    std::string hsv_mask_topic = camera_frame_id_ + "/image_hsv_mask";

    img_pub_ = image_transport::create_publisher(this, image_topic);
    info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(info_topic, 10);
    hsv_filtered_pub_ = image_transport::create_publisher(this, hsv_filtered_topic);
    hsv_mask_pub_ = image_transport::create_publisher(this, hsv_mask_topic);

    auto timer_period = std::chrono::milliseconds(1000 / std::max(1, framerate));
    timer_ = this->create_wall_timer(
      timer_period,
      std::bind(&OpenCVUsbCam::timer_callback, this));

    param_cb_handle_ = this->add_on_set_parameters_callback(
      std::bind(&OpenCVUsbCam::on_set_parameters, this, std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(),
      "OpenCV USB Camera node started (hsv_filter=%s, H[%d,%d] S[%d,%d] V[%d,%d])",
      enable_hsv_filter_ ? "true" : "false",
      h_min_, h_max_, s_min_, s_max_, v_min_, v_max_);
  }

  ~OpenCVUsbCam()
  {
    if (cap_.isOpened()) {
      cap_.release();
    }
  }

private:
  static int clamp_h(int value)
  {
    return std::max(0, std::min(179, value));
  }

  static int clamp_sv(int value)
  {
    return std::max(0, std::min(255, value));
  }

  rcl_interfaces::msg::SetParametersResult on_set_parameters(
    const std::vector<rclcpp::Parameter> & params)
  {
    auto result = rcl_interfaces::msg::SetParametersResult();
    result.successful = true;

    bool next_enable_hsv_filter = enable_hsv_filter_;
    bool next_publish_hsv_debug_topics = publish_hsv_debug_topics_;
    int next_h_min = h_min_;
    int next_h_max = h_max_;
    int next_s_min = s_min_;
    int next_s_max = s_max_;
    int next_v_min = v_min_;
    int next_v_max = v_max_;

    for (const auto & param : params) {
      const auto & name = param.get_name();

      try {
        if (name == "enable_hsv_filter") {
          next_enable_hsv_filter = param.as_bool();
        } else if (name == "publish_hsv_debug_topics") {
          next_publish_hsv_debug_topics = param.as_bool();
        } else if (name == "h_min") {
          next_h_min = clamp_h(static_cast<int>(param.as_int()));
        } else if (name == "h_max") {
          next_h_max = clamp_h(static_cast<int>(param.as_int()));
        } else if (name == "s_min") {
          next_s_min = clamp_sv(static_cast<int>(param.as_int()));
        } else if (name == "s_max") {
          next_s_max = clamp_sv(static_cast<int>(param.as_int()));
        } else if (name == "v_min") {
          next_v_min = clamp_sv(static_cast<int>(param.as_int()));
        } else if (name == "v_max") {
          next_v_max = clamp_sv(static_cast<int>(param.as_int()));
        }
      } catch (const std::exception & e) {
        result.successful = false;
        result.reason = std::string("Invalid parameter type for ") + name + ": " + e.what();
        return result;
      }
    }

    if (next_h_min > next_h_max || next_s_min > next_s_max || next_v_min > next_v_max) {
      result.successful = false;
      result.reason = "HSV min must be <= max for each channel";
      return result;
    }

    enable_hsv_filter_ = next_enable_hsv_filter;
    publish_hsv_debug_topics_ = next_publish_hsv_debug_topics;
    h_min_ = next_h_min;
    h_max_ = next_h_max;
    s_min_ = next_s_min;
    s_max_ = next_s_max;
    v_min_ = next_v_min;
    v_max_ = next_v_max;

    RCLCPP_INFO(
      this->get_logger(),
      "Updated HSV params: enabled=%s, debug_topics=%s, H[%d,%d] S[%d,%d] V[%d,%d]",
      enable_hsv_filter_ ? "true" : "false",
      publish_hsv_debug_topics_ ? "true" : "false",
      h_min_, h_max_, s_min_, s_max_, v_min_, v_max_);

    return result;
  }

  void timer_callback()
  {
    cv::Mat frame;
    if (!cap_.read(frame)) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Failed to grab frame");
      return;
    }

    std_msgs::msg::Header hdr;
    hdr.stamp = this->now();
    hdr.frame_id = camera_frame_id_;

    cv::Mat output_frame = frame;

    if (enable_hsv_filter_) {
      cv::Mat hsv;
      cv::Mat mask;
      cv::Mat filtered;

      cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
      cv::inRange(
        hsv,
        cv::Scalar(h_min_, s_min_, v_min_),
        cv::Scalar(h_max_, s_max_, v_max_),
        mask);
      cv::bitwise_and(frame, frame, filtered, mask);

      output_frame = filtered;

      if (publish_hsv_debug_topics_) {
        auto filtered_msg = cv_bridge::CvImage(hdr, "bgr8", filtered).toImageMsg();
        auto mask_msg = cv_bridge::CvImage(hdr, "mono8", mask).toImageMsg();
        hsv_filtered_pub_.publish(filtered_msg);
        hsv_mask_pub_.publish(mask_msg);
      }
    }

    auto img_msg = cv_bridge::CvImage(hdr, "bgr8", output_frame).toImageMsg();

    sensor_msgs::msg::CameraInfo ci = cim_->getCameraInfo();
    ci.header = hdr;

    img_pub_.publish(img_msg);
    info_pub_->publish(ci);
  }

  cv::VideoCapture cap_;
  std::string camera_frame_id_;
  std::shared_ptr<camera_info_manager::CameraInfoManager> cim_;

  bool enable_hsv_filter_{false};
  bool publish_hsv_debug_topics_{true};
  int h_min_{0};
  int h_max_{179};
  int s_min_{0};
  int s_max_{255};
  int v_min_{0};
  int v_max_{255};

  image_transport::Publisher img_pub_;
  image_transport::Publisher hsv_filtered_pub_;
  image_transport::Publisher hsv_mask_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OpenCVUsbCam>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
