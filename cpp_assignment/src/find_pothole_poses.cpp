// Resources Used:
//   - https://github.com/fmrico/book_ros2
//   - https://docs.opencv.org/4.x/index.html
//   - https://docs.ros.org/en/humble/Concepts/Basic/About-Interfaces.html
// 

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "pothole_interfaces/msg/image_coordinate.hpp"
#include "pothole_interfaces/msg/image_coordinate_array.hpp"
#include "tf2/LinearMath/Vector3.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/convert.h"
#include "tf2_ros/transform_listener.h"
#include <image_transport/image_transport.hpp>
#include <image_geometry/pinhole_camera_model.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

using std::placeholders::_1;

// Node used to subscribe to depth image + tf and publish poses in odom frame for potholes. 
// Works for both yolo detected potholes and cv detected potholes
// C++ used for faster + more stable tf lookup and transform

class FindPotholePoses : public rclcpp::Node
{
  public:
    FindPotholePoses()
    : Node("find_pothole_poses")
    {
      listener_ = std::make_shared<tf2_ros::TransformListener>(buffer_);
      coordinates_sub_ = this->create_subscription<pothole_interfaces::msg::ImageCoordinateArray>(
      "potholes_image_coordinate", 10, std::bind(&FindPotholePoses::image_coordinates_callback, this, _1));
      depth_image_sub_ = image_transport::create_subscription(this, 
      "/limo/depth_camera_link/depth/image_raw", std::bind(&FindPotholePoses::depth_image_callback, this, _1),
      "raw", rclcpp::SensorDataQoS().get_rmw_qos_profile());
      camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>("/limo/depth_camera_link/camera_info", 10, 
      std::bind(&FindPotholePoses::camera_info_callback, this, _1));
      pothole_poses_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/pothole_poses", 10);
    }

  private:
    void image_coordinates_callback(const pothole_interfaces::msg::ImageCoordinateArray & msg) const
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
      cv::Mat & depth_image_src = depth_image_->image;

      for(const auto& coordinate : msg.coordinates) {
        // RCLCPP_INFO(this->get_logger(), "X: %ld, Y: %ld", coordinate.x, coordinate.y);
        float depth = depth_image_src.at<float>(cv::Point2d(coordinate.x, coordinate.y));
        cv::circle(depth_image_src, cv::Point(coordinate.x, coordinate.y), 7, cv::Scalar(0, 0, 0), -1);
        if (!std::isnan(depth)) {
            if (depth > 0.4 && depth < 1.2) {
                cv::Point3d ray = model_.projectPixelTo3dRay(model_.rectifyPoint(cv::Point(coordinate.x, coordinate.y)));
                ray = ray / ray.z;
                cv::Point3d point = ray * depth;
                geometry_msgs::msg::TransformStamped image2target_msg;
                tf2::Stamped<tf2::Transform> image2target;

                image2target_msg = buffer_.lookupTransform(
                "odom", "depth_link", tf2::timeFromSec(0));
                tf2::fromMsg(image2target_msg, image2target);

                tf2::Vector3 point_tf(point.x, point.y, point.z);
                tf2::Vector3 p_bf = image2target * point_tf;
                geometry_msgs::msg::Pose pothole_pose;
                pothole_pose.position.x = p_bf.getX();
                pothole_pose.position.y = p_bf.getY();
                pothole_pose.position.z = p_bf.getZ();
                pothole_pose.orientation.w = 1.0;
                if (p_bf.getZ() < 0.3) {
                  message.poses.push_back(pothole_pose);
                  RCLCPP_INFO(this->get_logger(), "Coordinates: X: %lf, Y: %lf, Z: %lf", p_bf.getX(), p_bf.getY(), p_bf.getZ());
                }
            }
        }
      }
      pothole_poses_pub_->publish(message);
      cv::imshow("Depth Camera", depth_image_src);
      cv::waitKey(1);
    }

    void depth_image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
    {
      depth_image_ = cv_bridge::toCvCopy(msg, msg->encoding);
    }

    void camera_info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg)
    {
      RCLCPP_INFO(this->get_logger(), "Got Camera Info");
      model_.fromCameraInfo(msg);
      camera_info_sub_.reset();
    }

    rclcpp::Subscription<pothole_interfaces::msg::ImageCoordinateArray>::SharedPtr coordinates_sub_;
    image_transport::Subscriber depth_image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pothole_poses_pub_;
    image_geometry::PinholeCameraModel model_;
    cv_bridge::CvImagePtr depth_image_;
    tf2::BufferCore buffer_;
    std::shared_ptr<tf2_ros::TransformListener> listener_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FindPotholePoses>());
  rclcpp::shutdown();
  return 0;
}