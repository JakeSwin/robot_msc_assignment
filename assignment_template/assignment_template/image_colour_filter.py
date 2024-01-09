#!/bin/python3

import cv2
import numpy as np
import rclpy
import image_geometry
import matplotlib.pyplot as plt

from rclpy import qos
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from cv_bridge import CvBridge, CvBridgeError
from tf2_ros import Buffer, TransformListener
from sensor_msgs.msg import Image, CameraInfo
from tf2_geometry_msgs import do_transform_pose

class ImageColourFilter(Node):
    camera_model = None
    image_depth_ros = None

    # aspect ration between color and depth cameras
    # calculated as (color_horizontal_FOV/color_width) / (depth_horizontal_FOV/depth_width) from the dabai camera parameters
    color2depth_aspect = (71.0/640) / (67.9/400)

    def __init__(self):
        super().__init__("image_colour_filter")
        self.pose_array = PoseArray()
        self.pose_array.header.frame_id = "odom"
        self.poses = []

        self.image_sub = self.create_subscription(Image, 
                                                  "/limo/depth_camera_link/image_raw",
                                                  self.image_callback,
                                                  qos_profile=qos.qos_profile_sensor_data) # Set QoS Profile
        self.depth_image_sub = self.create_subscription(Image, '/limo/depth_camera_link/depth/image_raw', 
                                                  self.image_depth_callback, qos_profile=qos.qos_profile_sensor_data)
        self.camera_info_sub = self.create_subscription(CameraInfo, 
                                                  "/limo/depth_camera_link/camera_info",
                                                  self.info_callback,
                                                  qos_profile=qos.qos_profile_sensor_data)
        self.image_publisher = self.create_publisher(Image, "/filtered_image", 10)
        self.pose_array_publisher = self.create_publisher(PoseArray, "/pothole_poses", 10)

        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        plt.ion()
        self.x = []
        self.y = []
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot()
        self.ax.set_xlim(-2.5, 2.5)
        self.ax.set_ylim(-2, 1)
        self.scatter = self.ax.scatter(self.x, self.y)
        

    def get_tf_transform(self, target_frame, source_frame):
        try:
            transform = self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
            return transform
        except Exception as e:
            self.get_logger().warning(f"Failed to lookup transform: {str(e)}")
            return None
        
    def tf_transform(self, pose_stamped, target_frame):
        try:
            output_pose_stamped = self.tf_buffer.transform(pose_stamped, target_frame)
            return output_pose_stamped
        except Exception as e:
            self.get_logger().warning(f"Failed to lookup transform: {str(e)}")
            return None

    def image_depth_callback(self, data):
        self.image_depth_ros = data

    def image_callback(self, data):
        if self.camera_model is None:
            return

        if self.image_depth_ros is None:
            return
        
        self.poses = []
        self.pose_array.poses.clear()

        self.get_logger().info("Image Message Received")

        try:
            image_color = self.bridge.imgmsg_to_cv2(data, "bgr8")
            image_depth = self.bridge.imgmsg_to_cv2(self.image_depth_ros, "32FC1")
        except CvBridgeError as e:
            print(e)

        hsv_image = cv2.cvtColor(image_color, cv2.COLOR_BGR2HSV)
        purple_filter = cv2.inRange(hsv_image, (100, 0, 0), (220, 255, 255))
        analysis = cv2.connectedComponentsWithStats(purple_filter, 4, cv2.CV_32S)
        (totalLabels, label_ids, values, centroid) = analysis
        for i, (x, y) in enumerate(centroid):
            # print(x)
            # print(y)
            # print(values[i][4])
            shape_size = values[i][4]
            if shape_size > 300 and shape_size < 100_000:
                x = int(x)
                y = int(y)
                cv2.circle(image_color, (x, y), 5, (255, 255, 255), -1)
                depth_coords = (
                    image_depth.shape[0]/2 + 
                    (y - image_color.shape[0]/2) * 
                    self.color2depth_aspect, 
                    image_depth.shape[1]/2 + 
                    (x - image_color.shape[1]/2) * 
                    self.color2depth_aspect
                )
                depth_value = image_depth[int(depth_coords[0]), int(depth_coords[1])]

                camera_coords = self.camera_model.projectPixelTo3dRay((x, y))
                camera_coords = [x/camera_coords[2] for x in camera_coords]
                camera_coords = [x*depth_value for x in camera_coords]

                location = PoseStamped()
                location.header.frame_id = "depth_link"
                location.header.stamp = rclpy.time.Time().to_msg()
                location.pose.orientation.w = 1.0
                location.pose.position.x = camera_coords[0]
                location.pose.position.y = camera_coords[1]
                location.pose.position.z = camera_coords[2]

                # transform = self.get_tf_transform('depth_link', 'odom')
                # p_camera = do_transform_pose(location.pose, transform)

                odom_pose_stamped = self.tf_transform(location, "odom")

                print('odom coords: ', odom_pose_stamped)

                # publish_pose = Pose()
                # publish_pose.position.x = -p_camera.position.y
                # publish_pose.position.y = -p_camera.position.z
                # # publish_pose.position.z = p_camera.position.z
                # publish_pose.orientation.w = 1.0
                self.pose_array.poses.append(odom_pose_stamped.pose)

                self.x.append(odom_pose_stamped.pose.position.x)
                self.y.append(odom_pose_stamped.pose.position.y)
                self.scatter = self.ax.scatter(self.x, self.y)
                self.fig.canvas.draw()
                self.fig.canvas.flush_events()
                # cv2.circle(image_color, (int(x), int(y)), 10, (200, 255, 255), -1)
        # (totalLabels, label_ids, values, centroid) = analysis
        # output = np.zeros(hsv_image.shape, dtype="uint8") 
  
        # # Loop through each component 
        # for i in range(1, totalLabels): 
            
        #     # Area of the component 
        #     area = values[i, cv2.CC_STAT_AREA]  
            
        #     if (area > 140) and (area < 400): 
        #         componentMask = (label_ids == i).astype("uint8") * 255
        #         output = cv2.bitwise_or(output, componentMask) 
        
        # cv2.imshow("Filtered Components", output) 

        # contours, _ = cv2.findContours(purple_filter, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # for c in contours:
        #     M = cv2.moments(c)
        #     if M['m00'] > 300:
        #         cX = int(M["m10"] / M["m00"]) # image_coords[1]
        #         cY = int(M["m01"] / M["m00"]) # image_coords[0]
        #         cv2.circle(image_color, (cX, cY), 5, (255, 255, 255), -1)
        #         depth_coords = (
        #             image_depth.shape[0]/2 + 
        #             (cY - image_color.shape[0]/2) * 
        #             self.color2depth_aspect, 
        #             image_depth.shape[1]/2 + 
        #             (cX - image_color.shape[1]/2) * 
        #             self.color2depth_aspect
        #         )
        #         depth_value = image_depth[int(depth_coords[0]), int(depth_coords[1])]

        #         camera_coords = self.camera_model.projectPixelTo3dRay((cX, cY))
        #         camera_coords = [x/camera_coords[2] for x in camera_coords]
        #         camera_coords = [x*depth_value for x in camera_coords]

        #         location = PoseStamped()
        #         location.header.frame_id = "depth_link"
        #         location.header.stamp = rclpy.time.Time().to_msg()
        #         location.pose.orientation.w = 1.0
        #         location.pose.position.x = camera_coords[0]
        #         location.pose.position.y = camera_coords[1]
        #         location.pose.position.z = camera_coords[2]

        #         # transform = self.get_tf_transform('depth_link', 'odom')
        #         # p_camera = do_transform_pose(location.pose, transform)

        #         odom_pose_stamped = self.tf_transform(location, "odom")

        #         print('odom coords: ', odom_pose_stamped)

        #         # publish_pose = Pose()
        #         # publish_pose.position.x = -p_camera.position.y
        #         # publish_pose.position.y = -p_camera.position.z
        #         # # publish_pose.position.z = p_camera.position.z
        #         # publish_pose.orientation.w = 1.0
        #         self.pose_array.poses.append(odom_pose_stamped.pose)

        #         self.x.append(odom_pose_stamped.pose.position.x)
        #         self.y.append(odom_pose_stamped.pose.position.y)
        #         self.scatter = self.ax.scatter(self.x, self.y)
        #         self.fig.canvas.draw()
        #         self.fig.canvas.flush_events()
                

        apply_mask = cv2.bitwise_and(image_color, image_color, mask=purple_filter)
        apply_mask = cv2.cvtColor(apply_mask, cv2.COLOR_HSV2RGB)

        cv2.imshow("image depth", apply_mask)
        cv2.waitKey(1)

        self.pose_array_publisher.publish(self.pose_array)
        # result = self.bridge.cv2_to_imgmsg(apply_mask, "rgb8")
        # self.camera_model.projectPixelTo3dRay
        # self.image_publisher.publish(result)

    def info_callback(self, data):
        if not self.camera_model:
            self.camera_model = image_geometry.PinholeCameraModel()
        self.camera_model.fromCameraInfo(data)
        if self.destroy_subscription(self.camera_info_sub):
            self.get_logger().info("Sucessfully destroyed camera info sub")
        else:
            self.get_logger().info("Failed to destroy camera info sub, will try again")

def main(args=None):
    rclpy.init(args=args)
    image_colour_filter = ImageColourFilter()
    rclpy.spin(image_colour_filter)

    image_colour_filter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
