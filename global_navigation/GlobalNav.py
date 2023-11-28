# Importing libraries
from PIL import Image
import cv2
from IPython.display import Image, display
import math
import numpy as np
import matplotlib.pyplot as plt
from itertools import combinations
import colorsys
import pyvisgraph as vg
import time

class GlobalNav:
    def __init__(self):
        self._init_camera_QRdetector(1)
        self.obstacle_vertices, self.goal_center, self.transformation_matrix = self._init_background()

    def _init_camera_QRdetector(self, camera_id):
        self.video_stream = cv2.VideoCapture(camera_id)
        time.sleep(1)
        self.QR_detector = cv2.QRCodeDetectorAruco()
        if not self.video_stream.isOpened():
            raise IOError("Cannot open webcam")

    def _find_transformation_matrix(self):
        successInit = False
        background_found = False
        transformation_matrix_found = False
        while not successInit:
            image_detected, image = self.video_stream.read()
            if image_detected:
                image_initial = image.copy()
                height, width, channels = image.shape
                # this might be needed because the transformation matrix isnt always that good
                transformation_matrix = self.perspective_transformation(image)
                # Apply the new percpective on the frame
                if not transformation_matrix_found:
                    transformation_matrix = self.perspective_transformation(image)
                    transformation_matrix_found = True
                if transformation_matrix_found:
                    new_perspective_image = cv2.warpPerspective(image, transformation_matrix, (width, height))
                else:
                    new_perspective_image = image
                if not background_found:
                    #             plt.imshow(new_perspective_image)
                    #             plt.title("new perspective")
                    #             plt.show()
                    #                 obstacle_vertices, obstacle_edges, num_obstacles, goal_center = process_background(image_initial)
                    #                 if (num_obstacles != 2):
                    #                     continue
                    #                 print("here", obstacle_vertices)
                    #                 obstacle_vertices = np.array(obstacle_vertices, dtype=float)
                    #                 obstacle_vertices = cv2.perspectiveTransform(obstacle_vertices.reshape(-1, 1, 2), transformation_matrix)
                    #                 obstacle_vertices = obstacle_vertices.reshape(1, 8, 2)
                    #                 print(obstacle_vertices)

                    obstacle_vertices, obstacle_edges, num_obstacles, goal_center = self.process_background(
                        new_perspective_image)
                    print('vertices', obstacle_vertices)
                    # print('edges', obstacle_edges)
                    print('goal', goal_center)
                    successInit = (num_obstacles == 2 and goal_center is not None)
                    print('background found', background_found)

    def _init_background(self):

        while not successInit:
            image_detected, image = self.video_stream.read()
            if image_detected:
                image_initial = image.copy()
                height, width, channels = image.shape
                # this might be needed because the transformation matrix isnt always that good
                transformation_matrix = self.perspective_transformation(image)
                # Apply the new percpective on the frame
                if not transformation_matrix_found:
                    transformation_matrix = self.perspective_transformation(image)
                    transformation_matrix_found = True
                if transformation_matrix_found:
                    new_perspective_image = cv2.warpPerspective(image, transformation_matrix, (width, height))
                else:
                    new_perspective_image = image
                if not background_found:
                    #             plt.imshow(new_perspective_image)
                    #             plt.title("new perspective")
                    #             plt.show()
                    #                 obstacle_vertices, obstacle_edges, num_obstacles, goal_center = process_background(image_initial)
                    #                 if (num_obstacles != 2):
                    #                     continue
                    #                 print("here", obstacle_vertices)
                    #                 obstacle_vertices = np.array(obstacle_vertices, dtype=float)
                    #                 obstacle_vertices = cv2.perspectiveTransform(obstacle_vertices.reshape(-1, 1, 2), transformation_matrix)
                    #                 obstacle_vertices = obstacle_vertices.reshape(1, 8, 2)
                    #                 print(obstacle_vertices)

                    obstacle_vertices, obstacle_edges, num_obstacles, goal_center = self.process_background(
                        new_perspective_image)
                    print('vertices', obstacle_vertices)
                    # print('edges', obstacle_edges)
                    print('goal', goal_center)
                    successInit = (num_obstacles == 2 and goal_center is not None)
                    print('background found', background_found)

        return obstacle_vertices, goal_center, transformation_matrix

    def show_window(self, window_name = "thymio window"):
        image_detected, image = self.video_stream.read()
        if image_detected:
            cv2.imshow(window_name, image)