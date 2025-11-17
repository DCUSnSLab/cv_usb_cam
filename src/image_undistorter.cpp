// src/image_undistorter.cpp
#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

class ImageUndistorter : public rclcpp::Node
{
public:
  ImageUndistorter() : Node("image_undistorter")
  {
    /* ---------- 1. 파라미터 ---------- */
    this->declare_parameter("image_topic", "/camera/image_raw");
    this->declare_parameter("undistorted_topic", "/camera/image_undistorted");
    this->declare_parameter("camera_info_url", "");
    this->declare_parameter("camera_frame_id", "camera");

    std::string raw_topic = this->get_parameter("image_topic").as_string();
    std::string rect_topic = this->get_parameter("undistorted_topic").as_string();
    std::string info_url = this->get_parameter("camera_info_url").as_string();
    std::string frame_id = this->get_parameter("camera_frame_id").as_string();
    std::string info_topic = rect_topic + "/camera_info";

    /* ---------- 2. CameraInfo 로드 ---------- */
    cim_ = std::make_shared<camera_info_manager::CameraInfoManager>(
      this, "undistorter", info_url);
    cim_->setCameraName(frame_id);

    if (!info_url.empty() && (!cim_->validateURL(info_url) || !cim_->loadCameraInfo(info_url))) {
      RCLCPP_WARN(this->get_logger(), "Failed to load camera info from %s", info_url.c_str());
    }
    sensor_msgs::msg::CameraInfo raw_ci = cim_->getCameraInfo();

    /* ---------- 3. 언디스토션 맵 / 새 CameraInfo ---------- */
    cv::Mat K(3, 3, CV_64F, const_cast<double*>(raw_ci.k.data()));
    cv::Mat D((int)raw_ci.d.size(), 1, CV_64F, const_cast<double*>(raw_ci.d.data()));
    cv::Size sz(raw_ci.width, raw_ci.height);

    // 새 내부 파라미터
    cv::Mat newK = cv::getOptimalNewCameraMatrix(K, D, sz, 0);   // alpha=0 → 검은 테두리 최소
    cv::initUndistortRectifyMap(K, D, cv::Mat(), newK, sz, CV_16SC2, map1_, map2_);

    // 보정된 CameraInfo 미리 준비
    rect_ci_ = raw_ci;               // 뼈대 복사
    rect_ci_.header.frame_id = frame_id;
    rect_ci_.d.clear();              // 왜곡계수 제거
    rect_ci_.distortion_model = "plumb_bob";
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        rect_ci_.k[r * 3 + c] = newK.at<double>(r, c);   // 새 K 채워넣기
      }
    }
    // R = I
    rect_ci_.r = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    // P (3×4) 앞 3×3에 newK, 나머지 0
    rect_ci_.p = {newK.at<double>(0, 0), newK.at<double>(0, 1), newK.at<double>(0, 2), 0,
                  0,                      newK.at<double>(1, 1), newK.at<double>(1, 2), 0,
                  0,                      0,                      1,                      0};

    /* ---------- 4. Pub/Sub ---------- */
    sub_ = image_transport::create_subscription(
      this, raw_topic,
      std::bind(&ImageUndistorter::image_callback, this, std::placeholders::_1),
      "raw");

    img_pub_ = image_transport::create_publisher(this, rect_topic);
    info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(info_topic, 10);

    RCLCPP_INFO_STREAM(this->get_logger(),
                       "Undistorter ready\n  sub : " << raw_topic <<
                       "\n  pub : " << rect_topic <<
                       "\n  info: " << info_topic);
  }

private:
  /* ----- 콜백 ----- */
  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
  {
    try {
      cv::Mat raw = cv_bridge::toCvShare(msg, "bgr8")->image;
      cv::Mat rect;
      cv::remap(raw, rect, map1_, map2_, cv::INTER_LINEAR);

      // Image
      sensor_msgs::msg::Image::SharedPtr out =
        cv_bridge::CvImage(msg->header, "bgr8", rect).toImageMsg();
      img_pub_.publish(out);

      // CameraInfo (동일 헤더 타임스탬프로)
      rect_ci_.header.stamp = msg->header.stamp;
      info_pub_->publish(rect_ci_);
    }
    catch (const std::exception& e) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Undistort error: " << e.what());
    }
  }

  /* ----- 멤버 ----- */
  image_transport::Subscriber sub_;
  image_transport::Publisher img_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;

  std::shared_ptr<camera_info_manager::CameraInfoManager> cim_;
  sensor_msgs::msg::CameraInfo rect_ci_;

  cv::Mat map1_, map2_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ImageUndistorter>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
