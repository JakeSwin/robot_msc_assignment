#include <memory> 

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "tf2/LinearMath/Vector3.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/convert.h"
#include "tf2_ros/transform_listener.h"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <image_transport/image_transport.hpp>
#include <image_geometry/pinhole_camera_model.h>
#include <cv_bridge/cv_bridge.h>
using std::placeholders::_1;

// File not used in final version of code, used to illustrate a monolithic ros node design

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
      listener_ = std::make_shared<tf2_ros::TransformListener>(buffer_);
      image_sub_ = image_transport::create_subscription(this, 
      "/limo/depth_camera_link/image_raw", std::bind(&MinimalSubscriber::image_callback, this, _1),
      "raw", rclcpp::SensorDataQoS().get_rmw_qos_profile());
      depth_image_sub_ = image_transport::create_subscription(this, 
      "/limo/depth_camera_link/depth/image_raw", std::bind(&MinimalSubscriber::depth_image_callback, this, _1),
      "raw", rclcpp::SensorDataQoS().get_rmw_qos_profile());
      camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>("/limo/depth_camera_link/camera_info", 10, 
      std::bind(&MinimalSubscriber::camera_info_callback, this, _1));
      pothole_poses_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/pothole_poses", 10);
    }

  private:
    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg) const
    {
      if (depth_image_.get() == nullptr) {
        RCLCPP_INFO(this->get_logger(), "No Depth Image Yet");
        return;
      }
      if (model_.cameraInfo() == sensor_msgs::msg::CameraInfo()) {
        RCLCPP_INFO(this->get_logger(), "No Camera Info Yet");
        return;
      }
      geometry_msgs::msg::PoseArray message;
      message.header.frame_id = "odom";
      cv_bridge::CvImagePtr cv_ptr;
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      cv::Mat & image_src = cv_ptr->image;
      cv::Mat & depth_image_src = depth_image_->image;
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
          // Image Projection stuff that didn't work well so ignoring it
          // int depth_x = floor(depth_image_src.cols / 2 + (cy - image_src.cols / 2) * color2depth_aspect);
          // int depth_y = floor(depth_image_src.rows / 2 + (cx - image_src.rows / 2) * color2depth_aspect);
          // RCLCPP_INFO(this->get_logger(), "Depth [%d]: x: %d, y: %d", i, depth_x, depth_y);
          // cv::circle(depth_image_src, cv::Point(depth_x, depth_y), 7, cv::Scalar(0, 0, 0), -1);
          float depth = depth_image_src.at<float>(cv::Point2d(cx, cy));
          cv::circle(depth_image_src, cv::Point(cx, cy), 7, cv::Scalar(0, 0, 0), -1);
          if (!std::isnan(depth)) {
            if (depth > 0.4 && depth < 1.2) {
              // RCLCPP_INFO(this->get_logger(), "Depth [%d]: %g, Image Type: %d", i, depth, depth_image_src.type());
              cv::Point3d ray = model_.projectPixelTo3dRay(model_.rectifyPoint(cv::Point(cx, cy)));
              ray = ray / ray.z;
              cv::Point3d point = ray * depth;
              geometry_msgs::msg::TransformStamped image2target_msg;
              tf2::Stamped<tf2::Transform> image2target;

              image2target_msg = buffer_.lookupTransform(
                "odom", msg->header.frame_id, tf2::timeFromSec(0));
              tf2::fromMsg(image2target_msg, image2target);

              tf2::Vector3 point_tf(point.x, point.y, point.z);
              tf2::Vector3 p_bf = image2target * point_tf;
              geometry_msgs::msg::Pose pothole_pose;
              pothole_pose.position.x = p_bf.getX();
              pothole_pose.position.y = p_bf.getY();
              pothole_pose.position.z = p_bf.getZ();
              pothole_pose.orientation.w = 1.0;
              message.poses.push_back(pothole_pose);
              RCLCPP_INFO(this->get_logger(), "Coordinates [%d]: X: %lf, Y: %lf, Z: %lf", i, p_bf.getX(), p_bf.getY(), p_bf.getZ());
            }
          }
        }
      }
      pothole_poses_pub_->publish(message);
      cv::imshow("Camera", image_src);
      cv::imshow("Depth Camera", depth_image_src);
      cv::waitKey(1);
    }

    void depth_image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
    {
      // RCLCPP_INFO(this->get_logger(), "Got Depth Image");
      depth_image_ = cv_bridge::toCvCopy(msg, msg->encoding);
    }

    void camera_info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg)
    {
      RCLCPP_INFO(this->get_logger(), "Got Camera Info");
      model_.fromCameraInfo(msg);
      camera_info_sub_.reset();
    }

    image_transport::Subscriber image_sub_;
    image_transport::Subscriber depth_image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pothole_poses_pub_;
    image_geometry::PinholeCameraModel model_;
    cv_bridge::CvImagePtr depth_image_;
    tf2::BufferCore buffer_;
    std::shared_ptr<tf2_ros::TransformListener> listener_;
    const float color2depth_aspect = (71.0/640) / (67.9/400);
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}