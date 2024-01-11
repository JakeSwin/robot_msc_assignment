#!/bin/python3

import os
import rclpy
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cbook as cbook
import matplotlib.image as image

from functools import partial
from rclpy.node import Node
from geometry_msgs.msg import PoseArray

# TODO Add in a count for each cluster, filter out clusters that have too far below standard deviation
class Clusterer():
    def __init__(self):
        self.clusters = np.empty((0,2))
        # 0.3 - 14
        # 0.2 - 16
        # 0.1 - 19
        self.distance_threshold = 0.15
    
    def add_points(self, points):
        input_shape = np.shape(points)
        if len(input_shape) != 2 or input_shape[1] != 2:
            print("Data is in an invalid shape, expecting (x, 2)")
            return
        
        for point in points:
            distance_f = partial(self.distance, point)
            distances = list(map(distance_f, self.clusters))
            if distances:
                sorted = np.argsort(distances)
                closest_i = sorted[0]
                closest_distance = distances[closest_i]
                if closest_distance < self.distance_threshold:
                    self.clusters[closest_i] = point
                else:
                    self.clusters = np.concatenate((self.clusters, [point]), axis=0)
            else:
                self.clusters = np.concatenate((self.clusters, [point]), axis=0)

    @staticmethod
    def distance(p1, p2):
        return np.sqrt(np.square(p1[0] - p2[0]) + np.square(p1[1] - p2[1]))

class PotholeClustering(Node):

    def __init__(self):
        super().__init__("pothole_clustering")

        self.sub = self.create_subscription(PoseArray, "/pothole_poses", self.potholes_callback, 10)
        self.image = None
        self.clustering = Clusterer()
        plt.ion()
        dir = os.path.dirname(__file__)
        potholes_image_path = os.path.join(dir, '../models/potholes/materials/textures/background_potholes.png')
        with cbook.get_sample_data(potholes_image_path) as image_file:
            self.image = plt.imread(image_file)

        self.fig = plt.figure()
        self.ax = self.fig.add_subplot()
        self.ax.imshow(self.image, extent=(-1.44, 1.44, -1.24, 0.13))
        self.ax.set_xlim(-2.5, 2.5)
        self.ax.set_ylim(-2, 1)

    def draw_plot(self):
        self.ax.cla()
        self.ax.imshow(self.image, extent=(-1.44, 1.44, -1.24, 0.13))
        self.ax.set_title(f"Number of Potholes: {len(self.clustering.clusters)}")
        self.scatter = self.ax.scatter(self.clustering.clusters[:,0], self.clustering.clusters[:,1], c="#81ff26")
        # self.ax.text(0.05, 0.05, f"Number of Potholes: {len(self.clustering.clusters)}", transform=self.ax.transAxes)
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        
    def potholes_callback(self, data):
        self.clustering.add_points([[pose.position.x, pose.position.y] for pose in data.poses])
        self.draw_plot()
                

def main(args=None):
    rclpy.init(args=args)
    pothole_clustering = PotholeClustering()
    rclpy.spin(pothole_clustering)

    pothole_clustering.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
