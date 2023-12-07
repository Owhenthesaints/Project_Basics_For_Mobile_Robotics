import time
import matplotlib.pyplot as plt
import cv2
import numpy as np
import pyvisgraph as vg

from dependencies.helper_functions import convert_angle

TRANSFORMED_IMAGE_NAME = "transformed image"
NORMAL_IMAGE_NAME = "normal image"
ANGLE_OFFSET = np.pi
DEMO_DOUBLE_SCREEN_NAME = "annotated and regular image"

####################### CODE TO HELP VISUALIZE OUTPUT ##############################
    #     #Display the image using matplotlib
    #     plt.imshow(mask)
    #     plt.title("mask")
    #     plt.axis('off')  # Turn off axis labels
    #     plt.show()
####################################################################################

def process_Green_square(image, min_blue, min_green, min_red, max_blue, max_green, max_red, kernel_size=5):
    # Taking a matrix of size 5 as the kernel
    kernel = np.ones((5, 5), np.uint8)

    # HSV (Hue, Saturation, Value): Separates the color information from the brightness information, making it robust to changes in lighting conditions
    hsv_frame = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # getting the mask image from the HSV image using threshold values
    mask = cv2.inRange(hsv_frame, (min_blue, min_green, min_red), (max_blue, max_green, max_red))
    mask_dilation = cv2.dilate(mask, kernel, iterations=1)
    mask_erosion = cv2.erode(mask_dilation, kernel, iterations=1)

    return mask_erosion


def process_image(image, min_blue, min_green, min_red, max_blue, max_green, max_red, kernel_size=5):
    lower_threshold = 100
    upper_threshold = 150
    aperture_size = 7
    kernel = np.ones((5, 5), np.uint8)

    # HSV (Hue, Saturation, Value): Separates the color information from the brightness information, making it robust to changes in lighting conditions
    hsv_frame = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # getting the mask image from the HSV image using threshold values
    mask = cv2.inRange(hsv_frame, (min_blue, min_green, min_red), (max_blue, max_green, max_red))
    mask_dilation = cv2.dilate(mask, kernel, iterations=1)
    mask_erosion = cv2.erode(mask_dilation, kernel, iterations=1)
    inverted_image = cv2.bitwise_not(mask_erosion)
    med_img = cv2.medianBlur(inverted_image, kernel_size)
    canny_img = cv2.Canny(med_img, lower_threshold, upper_threshold, apertureSize=aperture_size, L2gradient=True)
    dilated_edges = cv2.dilate(canny_img, kernel, iterations=1)

    return dilated_edges


# https://www.geeksforgeeks.org/perspective-transformation-python-opencv/
# https://note.nkmk.me/en/python-opencv-qrcode/
# https://docs.opencv.org/4.x/dd/d49/tutorial_py_contour_features.html
# https://theailearner.com/tag/cv2-minarearect/

def perspective_transformation(image):
    # Destination points for the matrix transformation
    # dest_corners =np.float32([(width, height), (0, height), (width, 0), (0, 0)])
    height, width, _ = image.shape
    dest_corners = np.float32([(0, height), (width, height), (0, 0), (width, 0)])

    # Initialize a list to store the centers of the detected objects
    centers = []

    # Mask values of the object to be detected
    # (min_blue, min_green, min_red) = (11, 61, 0)
    # (max_blue, max_green, max_red) = (77, 255, 255)

    # on Jiarui's computer
    (min_blue, min_green, min_red) = (11, 61, 55)
    (max_blue, max_green, max_red) = (77, 255, 255)

    processed_mask = process_Green_square(image, min_blue, min_green, min_red, max_blue, max_green, max_red)

    # extracting the contours of the object
    contours, _ = cv2.findContours(processed_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

    # sorting the contour based of area
    contours = sorted(contours, key=cv2.contourArea, reverse=True)

    # Take the top 4 contours
    top_contours = contours[:4]

    # Extract the 4 biggest contours wich are not having the same center
    for contour in top_contours:
        (x, y, w, h) = cv2.boundingRect(contour)
        center = (x + w // 2, y + h // 2)

        # Check if the center is not close to any existing centers
        if all(np.linalg.norm(np.array(center) - np.array(existing_center)) > 50 for existing_center in centers):
            centers.append(center)
            cv2.rectangle(image, (x - 15, y - 15), (x + w + 15, y + h + 15), (0, 255, 0), 4)

    if len(centers) == 4:
        center_points = np.float32(centers).reshape(-1, 1, 2)
        transformation_matrix = np.array(cv2.getPerspectiveTransform(center_points, dest_corners)).astype(np.float32)
        print("transformationMatrix: ", transformation_matrix)
        return transformation_matrix
    else:
        # Return the initial image if not enough contours are detected
        transformation_matrix = None
        return transformation_matrix


def orientation_angle(points):
    points_np = np.array(points[0], dtype=np.float32)

    # Calculate the centroid (center) of the robot
    robot_center = np.mean(points_np, axis=0)

    # Choose one vertex as a reference (e.g., the first vertex)
    right_front = points_np[0]
    left_front = points_np[3]
    center_front = ((right_front[0] + left_front[0]) / 2, (right_front[1] + left_front[1]) / 2)

    # Calculate the vector from the centroid to the reference vertex
    vector_to_reference = center_front - robot_center

    # Calculate the orientation angle in degrees in the range of -180 to 180 degrees
    angle = (np.arctan2(vector_to_reference[1], vector_to_reference[0]) * 180 / np.pi + 180) % 360 - 180

    return angle, robot_center


# Function to calculate mean angle over a window of frames
def calculate_mean_angle(angle_list):
    return sum(angle_list) / len(angle_list) if len(angle_list) > 0 else 0.0


def detect_shape(cnt):  # Function to determine type of polygon on basis of number of sides
    shape = 'unknown'
    peri = cv2.arcLength(cnt, True)
    vertices = cv2.approxPolyDP(cnt, 0.02 * peri, True)
    sides = len(vertices)
    # print('sides', sides)
    if (sides == 3):
        shape = 'triangle'
    elif (sides == 5):
        shape = 'pentagon'
    elif (sides == 8):
        shape = 'octagon'
    else:
        shape = 'circle'
    return shape


# INPUTS: a contour
def scale_contour(original_contour):
    # Get the bounding rectangle around the shape
    x, y, w, h = cv2.boundingRect(original_contour)

    # Calculate the center of the bounding rectangle
    center = ((x + w // 2), (y + h // 2))

    
    scale_factor = 1.7

    
    scaled_contour = np.array([[(point[0][0] - center[0]) * scale_factor + center[0],
                                    (point[0][1] - center[1]) * scale_factor + center[1]]
                                   for point in original_contour], dtype=np.int32)

    return scaled_contour

def illustrate_scaling(scaled_contour_image, contours, scaled = False):
    
    obstacle_vertices = []  # List to store vertices for each obstacle
    obstacle_edges = []  # List to store lines for each triangle

    for cnt in contours:
        # shape = detectShape(cnt)
        peri = cv2.arcLength(cnt, True)
        vertices = cv2.approxPolyDP(cnt, 0.02 * peri, True)
        sides = len(vertices)
        # print(sides)
    
        if sides == 4:
            if scaled:
                cnt = scale_contour(cnt)

            vertices = cv2.approxPolyDP(cnt, 0.02 * cv2.arcLength(cnt, True), True)
            obstacle = []  # Store vertices for each obstacle
            edges = []  # Store lines for each obstacle

            for i, vertex in enumerate(vertices):
                x, y = vertex[0]
                obstacle.append((x, y))

                # Calculate the index of the next vertex in the list (wrapping around to the first vertex if it's the last one)
                next_index = 0 if i == len(vertices) - 1 else i + 1
                next_vertex = vertices[next_index][0]

                # Append the current edge to the list of edges
                edges.append(((x, y), (next_vertex[0], next_vertex[1])))
                # drawing the scaled contour
                if scaled:
                    color = (255, 0, 0)
                    cv2.circle(scaled_contour_image, (x,y), 5, (255, 0, 0), 10, -1)
                else:
                    color = (0, 0, 0)
                cv2.line(scaled_contour_image, (x,y), (next_vertex[0], next_vertex[1]), color, 3)

            obstacle_vertices.append(obstacle)  # Append the vertices to the list
            obstacle_edges.append(edges)  # Append the edges to the list          

def process_obstacles(contours):
    obstacle_vertices = []  # List to store vertices for each obstacle
    obstacle_edges = []  # List to store lines for each obstacle
    num_obstacles = 0

    for cnt in contours:
        #shape = detectShape(cnt)
        peri = cv2.arcLength(cnt, True)
        vertices = cv2.approxPolyDP(cnt, 0.02 * peri, True)
        sides = len(vertices)
    
        if sides == 4:
            num_obstacles += 1
            cnt = scale_contour(cnt)

            vertices = cv2.approxPolyDP(cnt, 0.02 * cv2.arcLength(cnt, True), True)
            obstacle = []  # Store vertices for each obstacle
            edges = []  # Store lines for each obstacle

            for i, vertex in enumerate(vertices):
                x, y = vertex[0]
                obstacle.append((x, y))

                # Calculate the index of the next vertex in the list (wrapping around to the first vertex if it's the last one)
                next_index = 0 if i == len(vertices) - 1 else i + 1
                next_vertex = vertices[next_index][0]

                # Append the current edge to the list of edges
                edges.append(((x, y), (next_vertex[0], next_vertex[1])))

            obstacle_vertices.append(obstacle)  # Append the vertices to the list
            obstacle_edges.append(edges)  # Append the edges to the list
    return obstacle_vertices, obstacle_edges, num_obstacles


def process_goal(contours):
    goal_center = None

    for cnt in contours:
        peri = cv2.arcLength(cnt, True)
        vertices = cv2.approxPolyDP(cnt, 0.02 * peri, True)
        sides = len(vertices)
        # print('shape',shape)
        if sides == 8:
            # Store circle information
            (goal_center, radius) = cv2.minEnclosingCircle(cnt)
            goal_center = (int(goal_center[0]), int(goal_center[1]))
            radius = int(radius)

    return goal_center


def process_background(image):

    # Defining the the RGB threshold values for the obstacles
    (min_blue_obst, min_green_obst, min_red_obst) = (0, 0, 0)
    (max_blue_obst, max_green_obst, max_red_obst) = (255, 171, 29)

    # Defining the the RGB threshold values for the goal destination
    (min_blue_goal, min_green_goal, min_red_goal) = (97, 102, 71)
    (max_blue_goal, max_green_goal, max_red_goal) = (118, 192, 121)

    # Processing the obstacles to find the vertices and edges
    processed_obstacles = process_image(image, min_blue_obst, min_green_obst, min_red_obst, max_blue_obst,
                                        max_green_obst, max_red_obst)
    (obstacle_contours, _) = cv2.findContours(processed_obstacles, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    obstacle_vertices, obstacle_edges, num_obstacles = process_obstacles(obstacle_contours)
    
    # # visualize scaling contour
    # # Code used to get graphs for the report
    # height, width, _ = image.shape
    # scaled_contour_img = np.ones((height, width, 3), dtype = np.uint8) * 255
    # illustrate_scaling(scaled_contour_img, obstacle_contours, False)
    # illustrate_scaling(scaled_contour_img, obstacle_contours, True)
    
    # plt.imshow(scaled_contour_img)
    # plt.title("Plot of original contour and scaled contour")
    # plt.show()

    # Display the processed grayscale mask using matplotlib
    # plt.imshow(processed_obstacles, cmap='gray')
    # plt.title("goal")
    # plt.axis('off')
    # plt.show()

    # Processing the goal destination to find the center
    processed_goal = process_image(image, min_blue_goal, min_green_goal, min_red_goal, max_blue_goal, max_green_goal,
                                   max_red_goal)
    (goal_contours, _) = cv2.findContours(processed_goal, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    goal_center = process_goal(goal_contours)

    return obstacle_vertices, obstacle_edges, num_obstacles, goal_center


# INPUTS: vertices of obstacles, the robot position, goal position
def get_shortest_path(shape_vertices, rob_pos, goal_pos):
    polygons = []
    for shape in shape_vertices:
        polygon = []
        for point in shape:
            polygon.append(vg.Point(point[0], point[1]))
        polygons.append(polygon)

    graph = vg.VisGraph()
    graph.build(polygons)

    startPosition = vg.Point(rob_pos[0], rob_pos[1])
    endPosition = vg.Point(goal_pos[0], goal_pos[1])

    shortestPath = graph.shortest_path(startPosition, endPosition)
    # print(shortestPath)
    return shortestPath


def draw_path_on_camera(camera_image, shortest_path, obstacle_vertices, robot_center, robot_angle):
    # drawing the expanded_vertices
    for vertices in obstacle_vertices:
        for i, vertex in enumerate(vertices):
            x = vertex[0]
            y = vertex[1]
            cv2.circle(camera_image, (x, y), 10, (255, 0, 0), -1)

            # creating a list of edges from the list of vertices
    edgelist = []
    for i, node in enumerate(shortest_path[:-1]):
        edgelist.append((shortest_path[i], shortest_path[i + 1]))

    color = (0, 255, 255)
    thickness = 3

    # drawing the optimal path using edges
    for i, edge in enumerate(edgelist):
        # if the robot is already on the path of an edge, draw from that point
        cv2.line(camera_image, (int(edgelist[i][0].x), int(edgelist[i][0].y)),
                 (int(edgelist[i][1].x), int(edgelist[i][1].y)), color, thickness)

    # drawing start and goal positions
    goal_location = shortest_path[-1]
    cv2.circle(camera_image, (int(goal_location.x), int(goal_location.y)), 1, (255, 0, 0), -1)
    cv2.circle(camera_image, (int(robot_center[0]), int(robot_center[1])), 5, (0, 0, 255), -1)

    # draw a directional arrow of robot orientation
    length = 50
    endpoint_x = int(robot_center[0] + length * np.cos((robot_angle)))
    endpoint_y = int(robot_center[1] + length * np.sin((robot_angle)))
    cv2.arrowedLine(camera_image, (int(robot_center[0]), int(robot_center[1])), (endpoint_x, endpoint_y), (255, 255, 0), 2)

# function initializes the webcam
def init_camera_QRdetector(camera_id):
    video_stream = cv2.VideoCapture(camera_id)
    time.sleep(0.5)
    QR_detector = cv2.QRCodeDetector()
    if not video_stream.isOpened():
        raise IOError("Cannot open webcam")
    # discard 50 most recent images
    for i in range(50):
        video_stream.read()
    return video_stream, QR_detector


def kill_camera(video_stream):
    video_stream.release()
    cv2.destroyAllWindows()

def convertVG_to_np(point):
    return np.array([point.x, point.y], dtype = int)

def init_background(video_stream):
    successInit = False
    transformation_matrix_found = False
    transformation_matrix = None
    background_found = False
    num_obstacles = 0

    while not successInit:
        image_detected, image = video_stream.read()
        if image_detected:
            image_initial = image.copy()
            height, width, channels = image.shape
            # this might be needed because the transformation matrix isnt always that good
            transformation_matrix = perspective_transformation(image)
            if transformation_matrix is not None:
                # print(transformation_matrix)
                new_perspective_image = cv2.warpPerspective(image, transformation_matrix, (width, height))
            else:
                print("background not found")
                continue

            obstacle_vertices, obstacle_edges, num_obstacles, goal_center = process_background(new_perspective_image)
            print('vertices', obstacle_vertices)
            print('num obstacles', num_obstacles)
            # print('edges', obstacle_edges)
            print('goal', goal_center)
            successInit = (num_obstacles == 2 and goal_center is not None)
            print('background found', successInit)

    return obstacle_vertices, goal_center, transformation_matrix

def find_thymio_position_angle(triangle_contours):
    triangle_contours = np.array(triangle_contours)
    triangle_side_lengths = np.array([[np.linalg.norm(triangle_contours[0] - triangle_contours[1])],
                                      [np.linalg.norm(triangle_contours[1] - triangle_contours[2])],
                                      [np.linalg.norm(triangle_contours[2] - triangle_contours[0])]])
    # draw a line from middle of shortest side of triangle to the tip of the triangle
    # print(triangle_side_lengths)
    shortest_side_index = np.argmin(triangle_side_lengths)

    if shortest_side_index == 0:
        front = triangle_contours[2]
        middle = np.array([(triangle_contours[0][0] + triangle_contours[1][0]) / 2,
                           (triangle_contours[0][1] + triangle_contours[1][1]) / 2])
    elif shortest_side_index == 1:
        front = triangle_contours[0]
        middle = np.array([(triangle_contours[1][0] + triangle_contours[2][0]) / 2,
                           (triangle_contours[1][1] + triangle_contours[2][1]) / 2])
    else:
        front = triangle_contours[1]
        middle = np.array([(triangle_contours[2][0] + triangle_contours[0][0]) / 2,
                           (triangle_contours[2][1] + triangle_contours[0][1]) / 2])

    # find the center of the triangle
    center = np.array((front + middle) / 2)

    # find the angle of the triangle
    angle = np.arctan2(front[1] - middle[1], front[0] - middle[0])

    return center, angle


class GlobalNav2:
    __image = None
    __new_perspective_image = None
    __shortest_path = None
    __on_goal = False
    __goal_center = None
    __position_array: list[np.ndarray] = []

    def __init__(self):
        # self.__CAMERA_ID = 0
        # on Jiarui's computer
        self.__CAMERA_ID = 1
        self.__video_stream, self.QR_detector = init_camera_QRdetector(self.__CAMERA_ID)
        self.__obstacle_vertices, self.__goal_center, self.__transformation_matrix = init_background(
            self.__video_stream)
        self.__get_most_recent_image()
        self._position = None

    def __get_most_recent_image(self):
        detected, self.__image = self.__video_stream.read()
        if detected:
            self.__get_warped_perspective_image()
        return detected

    def find_thymio(self):
        print("finding thymio")
        # filter out red color to get triangle
        # Convert the frame from BGR to HSV
        self.__get_most_recent_image()

        hsv = cv2.cvtColor(self.__new_perspective_image.copy(), cv2.COLOR_BGR2HSV)

        # Define the range for red color in HSV
        # lower_red = np.array([0, 158, 66])
        # upper_red = np.array([18, 255, 255])

        # on Jiarui's computer
        lower_red = np.array([0, 158, 130])
        upper_red = np.array([252, 255, 255])

        # Create a mask for the red color
        # kernel = np.ones((5, 5), np.uint8)
        mask = cv2.inRange(hsv, lower_red, upper_red)

        # Find connected components with stats
        _, labels, stats, centroids = cv2.connectedComponentsWithStats(mask)

        try:
            # Get the index of the connected component with the largest area
            largest_component_index = np.argmax(stats[1:, cv2.CC_STAT_AREA]) + 1  # Skip the background component
        except ValueError as e:
            print("Thymio is not found")
            return False
        # Create a mask for the largest connected component
        largest_component_mask = (labels == largest_component_index).astype(np.uint8)

        # Find contours in the mask
        contours, _ = cv2.findContours(largest_component_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # Filter out triangles based on the number of vertices
        for contour in contours:
            approx = cv2.approxPolyDP(contour, 0.03 * cv2.arcLength(contour, True), True)
            if len(approx) == 3:
                cnt = np.squeeze(approx)
                position, angle = find_thymio_position_angle(cnt)
                angle = convert_angle(angle + ANGLE_OFFSET)
                self._position = np.array([position[0], position[1], -angle])
                return True
            else:
                print("cannot find thymio")
                return False

    def get_position_and_angle(self):
        return self._position

    def __get_warped_perspective_image(self):
        height, width, channels = self.__image.shape
        self.__new_perspective_image = cv2.warpPerspective(self.__image, self.__transformation_matrix,
                                                           (width, height))

    def calculate_global_navigation(self):
        if self.__get_most_recent_image() and self._position is not None:
            self.__shortest_path = get_shortest_path(self.__obstacle_vertices, self._position,
                                                     self.__goal_center)

    def get_on_goal(self):
        return self.__on_goal

    def override_position(self, position):
        self._position = position

    def append_position_to_history(self):
        self.__position_array.append(self._position)

    def show_image(self, transformed: bool = True, draw_path: bool = True, trajectory: bool = True,
                   double_screen: bool = True, uncertainty: bool = False, estimate=0, probability=0):
        if self.__get_most_recent_image():
            if draw_path:
                self.calculate_global_navigation()
                draw_path_on_camera(self.__new_perspective_image, self.__shortest_path, self.__obstacle_vertices,
                                    self._position[0:2], -(self._position[2] - ANGLE_OFFSET))
                
            if trajectory and len(self.__position_array) != 0:
                purple_bgr = (255, 0, 255)
                for position in self.__position_array:
                    cv2.circle(self.__new_perspective_image, (int(position[0]), int(position[1])), 5, purple_bgr,
                               -1)  # Red circles

            if uncertainty:
                pos = np.array(np.round(estimate[0:2]), dtype=int)
                angle = estimate[2]
                prob = np.array(np.round(np.sqrt(np.array([probability[1, 1], probability[0, 0]]))), dtype=int)

                # Using cv2.ellipse() method
                # Draw a ellipse with red line borders of thickness of 2 px
                cv2.ellipse(self.__new_perspective_image, pos, axes=prob, angle=angle, startAngle=0, endAngle=360,
                            color=(255, 100, 0), thickness=2)

            if transformed:
                if self.__new_perspective_image is not None:
                    if double_screen:
                        annotated_and_regular = np.concatenate((self.__image, self.__new_perspective_image), axis=0)
                        cv2.imshow(DEMO_DOUBLE_SCREEN_NAME, annotated_and_regular)
                    else:
                        cv2.imshow(TRANSFORMED_IMAGE_NAME, self.__new_perspective_image)
                else:
                    cv2.imshow(NORMAL_IMAGE_NAME, self.__image)
            else:
                cv2.imshow(NORMAL_IMAGE_NAME, self.__image)
        else:
            print("no image to show")
            return False

    def is_on_goal(self):
        distance_to_goal = np.sqrt(
            (self._position[0] - self.__goal_center[0]) ** 2 + (self._position[1] - self.__goal_center[1]) ** 2)
        self.__on_goal = distance_to_goal < 20

    def get_next_position(self):
        if len(self.__shortest_path) == 2:
            self.is_on_goal()
            vg_point_next_pos = self.__shortest_path[1]
            next_pos_to_go = np.array([vg_point_next_pos.x, vg_point_next_pos.y])
            next_angle = np.arctan2((next_pos_to_go[1] - self._position[1]), (next_pos_to_go[0] - self._position[0]))
            next_angle = convert_angle(next_angle)
            return np.array([next_pos_to_go[0], next_pos_to_go[1], next_angle])
        elif self.__shortest_path is not None:
            vg_point_next_pos = self.__shortest_path[1]
            next_pos_to_go = np.array([vg_point_next_pos.x, vg_point_next_pos.y])
            vg_point_next_next_pos = self.__shortest_path[2]
            next_next_pos_to_go = np.array([vg_point_next_next_pos.x, vg_point_next_next_pos.y])
            next_angle = np.arctan2(next_next_pos_to_go[1] - next_pos_to_go[1],
                                    next_next_pos_to_go[0] - next_pos_to_go[0])
            next_angle = convert_angle(next_angle)
            return np.array([next_pos_to_go[0], next_pos_to_go[1], next_angle])
        else:
            return None

    def __del__(self):
        cv2.destroyWindow(NORMAL_IMAGE_NAME)
        cv2.destroyWindow(TRANSFORMED_IMAGE_NAME)
        self.__video_stream.release()
