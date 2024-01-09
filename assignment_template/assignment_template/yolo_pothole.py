#!/bin/python3
# Resources Used: 
#   - https://docs.ultralytics.com/modes/predict/

import cv2
import rclpy

from rclpy import qos
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from pothole_interfaces.msg import ImageCoordinate, ImageCoordinateArray
from ultralytics import YOLO

class YoloPothole(Node):

    def __init__(self):
        super().__init__("yolo_pothole")
        self.coordinate_array = ImageCoordinateArray()

        self.image_sub = self.create_subscription(Image, 
                                                  "/limo/depth_camera_link/image_raw",
                                                  self.image_callback,
                                                  qos_profile=qos.qos_profile_sensor_data)
        self.image_coordinate_publisher = self.create_publisher(ImageCoordinateArray, "/potholes_image_coordinate", 10)

        self.bridge = CvBridge()
        self.model = YOLO("/home/swin/cmp9767_assignment_ws/src/assignment_template/assignment_template/best_custom_pothole_1.pt")

    def image_callback(self, data):
        try:
            image_color = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        self.coordinate_array.coordinates.clear()

        results = self.model(image_color)
        for result in results:
            if result.boxes:
                for i, box in enumerate(result.boxes.xyxy):
                    x_center = int(box[0] + (box[2] - box[0]) / 2)
                    y_center = int(box[1] + (box[3] - box[1]) / 2)
                    cv2.rectangle(image_color, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])), (255, 255, 255), 3)
                    cv2.putText(image_color, str(result.boxes.conf[i]), (int(box[0]), int(box[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                    cv2.circle(image_color, (x_center, y_center), 2, (255, 255, 255), -1)

                    coordinate = ImageCoordinate()
                    coordinate.x = x_center
                    coordinate.y = y_center
                    self.coordinate_array.coordinates.append(coordinate)

        cv2.imshow("image depth", image_color)
        cv2.waitKey(1)

        self.image_coordinate_publisher.publish(self.coordinate_array)

def main(args=None):
    rclpy.init(args=args)
    yolo_pothole = YoloPothole()
    rclpy.spin(yolo_pothole)

    yolo_pothole.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
