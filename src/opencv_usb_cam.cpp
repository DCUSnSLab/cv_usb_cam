// src/opencv_usb_cam.cpp
#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/camera_info.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <opencv2/opencv.hpp>

class OpenCVUsbCam : public rclcpp::Node
{
public:
  OpenCVUsbCam() : Node("opencv_usb_cam")
  {
    // 1) 파라미터 선언 및 로드
    this->declare_parameter("video_device", "/dev/video0");
    this->declare_parameter("image_width", 640);
    this->declare_parameter("image_height", 480);
    this->declare_parameter("framerate", 30);
    this->declare_parameter("camera_frame_id", "camera");
    this->declare_parameter("camera_info_url", "");

    std::string video_device = this->get_parameter("video_device").as_string();
    int width = this->get_parameter("image_width").as_int();
    int height = this->get_parameter("image_height").as_int();
    int framerate = this->get_parameter("framerate").as_int();
    camera_frame_id_ = this->get_parameter("camera_frame_id").as_string();
    std::string camera_info_url = this->get_parameter("camera_info_url").as_string();

    // 2) VideoCapture 설정
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

    // 3) CameraInfoManager 선언 및 보정 파일 로드
    cim_ = std::make_shared<camera_info_manager::CameraInfoManager>(
      this, camera_frame_id_, camera_info_url);

    if (!camera_info_url.empty() && !cim_->loadCameraInfo(camera_info_url)) {
      RCLCPP_WARN(this->get_logger(), "Failed to load camera info from %s", camera_info_url.c_str());
    }

    // 4) 퍼블리셔 설정
    std::string image_topic = camera_frame_id_ + "/image_raw";
    std::string info_topic = camera_frame_id_ + "/camera_info";

    img_pub_ = image_transport::create_publisher(this, image_topic);
    info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(info_topic, 10);

    // 5) 타이머 설정 (프레임레이트에 맞춰)
    auto timer_period = std::chrono::milliseconds(1000 / framerate);
    timer_ = this->create_wall_timer(
      timer_period,
      std::bind(&OpenCVUsbCam::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "OpenCV USB Camera node started");
  }

  ~OpenCVUsbCam()
  {
    if (cap_.isOpened()) {
      cap_.release();
    }
  }

private:
  void timer_callback()
  {
    cv::Mat frame;
    if (!cap_.read(frame)) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "Failed to grab frame");
      return;
    }

    // 6) Image 메시지 변환
    std_msgs::msg::Header hdr;
    hdr.stamp = this->now();
    hdr.frame_id = camera_frame_id_;

    sensor_msgs::msg::Image::SharedPtr img_msg =
      cv_bridge::CvImage(hdr, "bgr8", frame).toImageMsg();

    // 7) CameraInfo 메시지 준비
    sensor_msgs::msg::CameraInfo ci = cim_->getCameraInfo();
    ci.header = hdr;

    // 8) 퍼블리시
    img_pub_.publish(img_msg);
    info_pub_->publish(ci);
  }

  cv::VideoCapture cap_;
  std::string camera_frame_id_;
  std::shared_ptr<camera_info_manager::CameraInfoManager> cim_;
  image_transport::Publisher img_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OpenCVUsbCam>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
