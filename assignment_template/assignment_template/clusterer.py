import numpy as np

from functools import partial

class Clusterer():
    def __init__(self):
        self.clusters = np.empty((0,2))
        self.distance_threshold = 0.3
    
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
        # self.clusters = np.concatenate((self.clusters, points), axis=0)

    @staticmethod
    def distance(p1, p2):
        return np.sqrt(np.square(p1[0] - p2[0]) + np.square(p1[1] - p2[1]))