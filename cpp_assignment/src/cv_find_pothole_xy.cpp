// Resources Used:
//   - https://github.com/fmrico/book_ros2
//   - https://docs.opencv.org/4.x/index.html
//   - https://docs.ros.org/en/humble/Concepts/Basic/About-Interfaces.html
// 

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "pothole_interfaces/msg/image_coordinate.hpp"
#include "pothole_interfaces/msg/image_coordinate_array.hpp"
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
using std::placeholders::_1;

// OpenCV alternative to yolo_pothole.py node, written in c++ for better speed and performance.

class CVFindPotholeXY : public rclcpp::Node
{
  public:
    CVFindPotholeXY()
    : Node("minimal_subscriber")
    {
      image_sub_ = image_transport::create_subscription(this, 
      "/limo/depth_camera_link/image_raw", std::bind(&CVFindPotholeXY::image_callback, this, _1),
      "raw", rclcpp::SensorDataQoS().get_rmw_qos_profile());
      pothole_poses_pub_ = this->create_publisher<pothole_interfaces::msg::ImageCoordinateArray>("/potholes_image_coordinate", 10);
    }

  private:
    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg) const
    {
      pothole_interfaces::msg::ImageCoordinateArray message;
      cv_bridge::CvImagePtr cv_ptr;
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      cv::Mat & image_src = cv_ptr->image;
      cv::Mat frame_hsv, frame_threshold, labels, stats, centroids;
      cv::cvtColor(image_src, frame_hsv, cv::COLOR_BGR2HSV);
      cv::inRange(frame_hsv, cv::Scalar(100, 0, 0), cv::Scalar(220, 255, 255), frame_threshold);
      int label_count = cv::connectedComponentsWithStats(frame_threshold, labels, stats, centroids, 8);
      for (int i = 0; i < label_count; i++)
      {
        int area = stats.at<int>(i, cv::CC_STAT_AREA);
        int cx = floor(centroids.at<double>(i, 0));
        int cy = floor(centroids.at<double>(i, 1));
        if (area > 300 && area < 100000) 
        {
          cv::circle(image_src, cv::Point(cx, cy), 7, cv::Scalar(211, 224, 31), -1);
          pothole_interfaces::msg::ImageCoordinate coordinate;
          coordinate.x = cx;
          coordinate.y = cy;
          message.coordinates.push_back(coordinate);
        }
      }
      pothole_poses_pub_->publish(message);
      cv::imshow("Camera", image_src);
      cv::waitKey(1);
    }

    // RGB image subscriber
    image_transport::Subscriber image_sub_;
    // Publisher
    rclcpp::Publisher<pothole_interfaces::msg::ImageCoordinateArray>::SharedPtr pothole_poses_pub_;
    cv_bridge::CvImagePtr depth_image_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CVFindPotholeXY>());
  rclcpp::shutdown();
  return 0;
}