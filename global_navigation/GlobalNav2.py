import time

import cv2
import matplotlib.pyplot as plt
import numpy as np
import pyvisgraph as vg
from dependencies.helper_functions import convert_angle

TRANSFORMED_IMAGE_NAME = "transformed image"
NORMAL_IMAGE_NAME = "normal image"


def process_Green_square(image, min_blue, min_green, min_red, max_blue, max_green, max_red, kernel_size=5):
    # Taking a matrix of size 5 as the kernel
    kernel = np.ones((5, 5), np.uint8)

    # HSV (Hue, Saturation, Value): Separates the color information from the brightness information, making it robust to changes in lighting conditions
    hsv_frame = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # getting the mask image from the HSV image using threshold values
    mask = cv2.inRange(hsv_frame, (min_blue, min_green, min_red), (max_blue, max_green, max_red))
    mask_dilation = cv2.dilate(mask, kernel, iterations=1)
    mask_erosion = cv2.erode(mask_dilation, kernel, iterations=1)

    # Display the image using matplotlib
    # plt.imshow(mask_erosion)
    # plt.title("mask green")
    # plt.axis('off')  # Turn off axis labels
    # plt.show()

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
    #     #Display the image using matplotlib
    #     plt.imshow(mask)
    #     plt.title("mask")
    #     plt.axis('off')  # Turn off axis labels
    #     plt.show()
    mask_dilation = cv2.dilate(mask, kernel, iterations=1)
    mask_erosion = cv2.erode(mask_dilation, kernel, iterations=1)
    inverted_image = cv2.bitwise_not(mask_erosion)
    med_img = cv2.medianBlur(inverted_image, kernel_size)
    canny_img = cv2.Canny(med_img, lower_threshold, upper_threshold, apertureSize=aperture_size, L2gradient=True)
    dilated_edges = cv2.dilate(canny_img, kernel, iterations=1)

    #     #Display the image using matplotlib
    #     plt.imshow(dilated_edges)
    #     plt.title("mask")
    #     plt.axis('off')  # Turn off axis labels
    #     plt.show()

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
    (min_blue, min_green, min_red) = (0, 249, 85)
    (max_blue, max_green, max_red) = (50, 255, 153)

    processed_mask = process_Green_square(image, min_blue, min_green, min_red, max_blue, max_green, max_red)

    # extracting the contours of the object
    contours, _ = cv2.findContours(processed_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

    # sorting the contour based of area
    contours = sorted(contours, key=cv2.contourArea, reverse=True)

    # Take the top 4 contours
    top_contours = contours[:4]

    # print('number of contours', len(top_contours))

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
        transformation_matrix = cv2.getPerspectiveTransform(center_points, dest_corners)
        return transformation_matrix
    else:
        # Return the initial image if not enough contours are detected
        transformation_matrix = None
        return transformation_matrix


# Empty function
def doNothing(x):
    pass


def find_thresh(image):
    # creating a resizable window named Track Bars
    cv2.namedWindow('Track Bars', cv2.WINDOW_NORMAL)

    # creating track bars for gathering threshold values of red green and blue
    cv2.createTrackbar('min_blue', 'Track Bars', 0, 255, doNothing)
    cv2.createTrackbar('min_green', 'Track Bars', 0, 255, doNothing)
    cv2.createTrackbar('min_red', 'Track Bars', 0, 255, doNothing)

    cv2.createTrackbar('max_blue', 'Track Bars', 0, 255, doNothing)
    cv2.createTrackbar('max_green', 'Track Bars', 0, 255, doNothing)
    cv2.createTrackbar('max_red', 'Track Bars', 0, 255, doNothing)

    resized_image = cv2.resize(image, (800, 626))
    # converting into HSV color model
    hsv_image = cv2.cvtColor(resized_image, cv2.COLOR_BGR2HSV)

    # showing both resized and hsv image in named windows
    # cv2.imshow('Base Image', resized_image)
    # cv2.imshow('HSV Image', hsv_image)

    # creating a loop to get the feedback of the changes in trackbars
    while True:
        # reading the trackbar values for thresholds
        min_blue = cv2.getTrackbarPos('min_blue', 'Track Bars')
        min_green = cv2.getTrackbarPos('min_green', 'Track Bars')
        min_red = cv2.getTrackbarPos('min_red', 'Track Bars')

        max_blue = cv2.getTrackbarPos('max_blue', 'Track Bars')
        max_green = cv2.getTrackbarPos('max_green', 'Track Bars')
        max_red = cv2.getTrackbarPos('max_red', 'Track Bars')

        # using inrange function to turn on the image pixels where object threshold is matched
        mask = cv2.inRange(hsv_image, (min_blue, min_green, min_red), (max_blue, max_green, max_red))
        # showing the mask image
        cv2.imshow('Mask Image', mask)
        # checking if q key is pressed to break out of loop
        key = cv2.waitKey(25)
        if key == ord('q'):
            break

    # printing the threshold values for usage in detection application
    print(f'min_blue {min_blue}  min_green {min_green} min_red {min_red}')
    print(f'max_blue {max_blue}  max_green {max_green} max_red {max_red}')
    # destroying all windows
    cv2.destroyAllWindows()


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


def detectShape(cnt):  # Function to determine type of polygon on basis of number of sides
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


# INPUTS: a contour, and minimum distance in pixels needed to scale the contour
def scale_contour(original_contour, desired_min_distance):
    # Get the bounding rectangle around the shape
    x, y, w, h = cv2.boundingRect(original_contour)

    # Calculate the center of the bounding rectangle
    center = ((x + w // 2), (y + h // 2))

    scaled_adequate = False
    scale_factor = 1.3

    while (not scaled_adequate):
        # Scale each point of the contour relative to the center
        scaled_contour = np.array([[(point[0][0] - center[0]) * scale_factor + center[0],
                                    (point[0][1] - center[1]) * scale_factor + center[1]]
                                   for point in original_contour], dtype=np.int32)
        # checking if contour is scaled enough
        min_distance = float('inf')

        # print(original_contour)
        for point in scaled_contour:
            point = tuple(float(coord) for coord in point)
            distance = cv2.pointPolygonTest(original_contour, point, True)
            min_distance = min(min_distance, abs(distance))
        # print(min_distance)
        if (min_distance < desired_min_distance):
            scale_factor += 0.01
        #             print(scale_factor)
        else:
            scaled_adequate = True
            print("Adequate Scaling achieved for obstacles")

    return scaled_contour


def process_obstacles(contours):
    obstacle_vertices = []  # List to store vertices for each triangle
    obstacle_edges = []  # List to store lines for each triangle
    num_obstacles = 0

    for cnt in contours:
        # shape = detectShape(cnt)
        peri = cv2.arcLength(cnt, True)
        vertices = cv2.approxPolyDP(cnt, 0.02 * peri, True)
        sides = len(vertices)
        #print(sides)

        if sides == 4:
            num_obstacles += 1
            # print('shape',shape)
            minimum_distance = 10
            cnt = scale_contour(cnt, minimum_distance)

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
        shape = detectShape(cnt)
        # print('shape',shape)
        if shape == 'octagon':
            # Store circle information
            (goal_center, radius) = cv2.minEnclosingCircle(cnt)
            goal_center = (int(goal_center[0]), int(goal_center[1]))
            radius = int(radius)

    return goal_center


def process_background(image):
    # Scale_factor

    # Defining the the RGB threshold values for the obstacles
    (min_blue_obst, min_green_obst, min_red_obst) = (0, 150, 0)
    (max_blue_obst, max_green_obst, max_red_obst) = (255, 255, 74)

    # Defining the the RGB threshold values for the goal destination
    (min_blue_goal, min_green_goal, min_red_goal) = (0, 38, 0)
    (max_blue_goal, max_green_goal, max_red_goal) = (109, 97, 85)

    # Processing the obstacles to find the vertices and edges
    processed_obstacles = process_image(image, min_blue_obst, min_green_obst, min_red_obst, max_blue_obst,
                                        max_green_obst, max_red_obst)
    (obstacle_contours, _) = cv2.findContours(processed_obstacles, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    obstacle_vertices, obstacle_edges, num_obstacles = process_obstacles(obstacle_contours)

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

    # Display the processed grayscale mask using matplotlib
    # plt.imshow(processed_goal, cmap='gray')
    # plt.title("goal")
    # plt.axis('off')
    # plt.show()

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


# INPUTS: vertices of obstacles, the robot position, goal position
def get_shortest_path(shape_vertices, Rob_pos, Goal_pos):
    polygons = []
    for shape in shape_vertices:
        polygon = []
        for point in shape:
            polygon.append(vg.Point(point[0], point[1]))
        polygons.append(polygon)

    graph = vg.VisGraph()
    graph.build(polygons)

    startPosition = vg.Point(Rob_pos[0], Rob_pos[1])
    endPosition = vg.Point(Goal_pos[0], Goal_pos[1])

    shortestPath = graph.shortest_path(startPosition, endPosition)
    # print(shortestPath)
    return shortestPath


# INPUTS: the shortestPath: a vector of vg.Point vertices
# shape_vertices: the vertices of expanded shapes
# pathImage: the image to draw the path
def drawPathGraph(shape_vertices, shortestPath, pathImage):
    # drawing points for each expanded vertex in the shape
    for vertices in shape_vertices:
        for i, vertex in enumerate(vertices):
            x = vertex[0]
            y = vertex[1]
            cv2.circle(pathImage, (x, y), 5, (255, 0, 0), -1)

            # creating a list of edges to store path into
    edgelist = []
    for i, node in enumerate(shortestPath[:-1]):
        print(shortestPath[i])
        edgelist.append((shortestPath[i], shortestPath[i + 1]))

    color = (0, 255, 255)
    thickness = 3
    for i, edge in enumerate(edgelist):
        # print(int(edgelist[i][0].x), int(edgelist[i][0].y))
        # transformedEdge = cv2.perspectiveTransform(int(edgelist[i][0].x), int(edgelist[i][0].y))
        cv2.line(pathImage, (int(edgelist[i][0].x), int(edgelist[i][0].y)),
                 (int(edgelist[i][1].x), int(edgelist[i][1].y)), color, thickness)

    # drawing start and goal positions as circles
    goal_location = shortestPath[-1]
    robot_location = shortestPath[0]
    cv2.circle(pathImage, (int(goal_location.x), int(goal_location.y)), 15, (255, 0, 0), 1)
    cv2.circle(pathImage, (int(robot_location.x), int(robot_location.y)), 15, (0, 0, 255), 1)


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
        # print(int(edgelist[i][0].x), int(edgelist[i][0].y))
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


def calibrateHSV(video_stream, CAMERA_ID=1):
    init_camera_QRdetector(CAMERA_ID)
    image_detected, image = video_stream.read()
    if image_detected:
        find_thresh(image)


# function initializes the webcam
def init_camera_QRdetector(camera_id):
    video_stream = cv2.VideoCapture(camera_id)
    time.sleep(2)
    QR_detector = cv2.QRCodeDetector()
    if not video_stream.isOpened():
        raise IOError("Cannot open webcam")
    return video_stream, QR_detector


def kill_camera(video_stream):
    video_stream.release()
    cv2.destroyAllWindows()


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
            # Apply the new percpective on the frame
            if not transformation_matrix_found:
                transformation_matrix = perspective_transformation(image)
                transformation_matrix_found = True
            if transformation_matrix_found:
                new_perspective_image = cv2.warpPerspective(image, transformation_matrix, (width, height))
            else:
                new_perspective_image = image
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

            obstacle_vertices, obstacle_edges, num_obstacles, goal_center = process_background(new_perspective_image)
            print('vertices', obstacle_vertices)
            print('num obstacles', num_obstacles)
            # print('edges', obstacle_edges)
            print('goal', goal_center)
            successInit = (num_obstacles == 2 and goal_center is not None)
            print('background found', successInit)

    return obstacle_vertices, goal_center, transformation_matrix

def get_robot_pos_angle(image, QR_detector):
    robot_angle = None
    robot_center = None
    qr_vertices = None
    window_size = 5
    QR_detected, qr_vertices, _ = QR_detector.detectAndDecode(image)
    if QR_detected:
        # qr_vertices = cv2.perspectiveTransform(qr_vertices.reshape(-1, 1, 2), transformation_matrix)
        # qr_vertices = qr_vertices.reshape(1, 4, 2)

        robot_angle, robot_center = orientation_angle(qr_vertices)
        print('robot center', robot_center)
        # print(f"Individual Angle: {angle} degrees")
        angle_window = []
        angle_window.append(robot_angle)
        if len(angle_window) == window_size:
            mean_angle = calculate_mean_angle(angle_window)
            # print(f"Mean Angle over {window_size} frames: {mean_angle} degrees")
            angle_window = []  # Reset the window for the next set of frames
    return robot_angle, robot_center, qr_vertices

## REFACTOR THIS CODE
def find_pos_angle(p):
    """
    Compute the medians of the triangle defined by p[0], p[1], p[2](lists of 2 elements)
    taking CV2 convention into account
    """

    #middle of traingle segment
    m = np.array([
    [(p[1][0] + p[2][0]) / 2, (p[1][1] + p[2][1]) / 2],
    [(p[2][0] + p[0][0]) / 2, (p[2][1] + p[0][1]) / 2],
    [(p[0][0] + p[1][0]) / 2, (p[0][1] + p[1][1]) / 2]])
    
    
    # length of medianes on x and y
    d = p - m

    # abs lenght of medianes
    l = d[:,0]**2 + d[:,1]**2

    i = np.argmax(l)
    angle = np.arctan2(d[i][1], d[i][0])
    return m[i], angle

class GlobalNav2:
    __image = None
    __qr_vertices = None
    __new_perspective_image = None
    __shortest_path = None
    __intermediary_tracker: int = 0
    __on_objective = False
    __last_robot_center = None
    __goal_center = None
    __robot_angle = None

    def __init__(self):
        self.__CAMERA_ID = 0
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
        hsv = cv2.cvtColor(self.__new_perspective_image, cv2.COLOR_BGR2HSV)

        # Define the range for red color in HSV
        lower_red = np.array([0, 100, 100])
        upper_red = np.array([10, 255, 255])

        # Create a mask for the red color
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.inRange(hsv, lower_red, upper_red)
        # mask_dilation = cv2.dilate(mask, kernel, iterations=1)
        # mask_erosion = cv2.erode(mask_dilation, kernel, iterations=1)
        
        # plt.imshow(mask_erosion)
        # plt.title("transformed")
        # plt.axis('off')  # Turn off axis labels
        # plt.show()

        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # Filter out triangles based on the number of vertices
        for contour in contours:
            approx = cv2.approxPolyDP(contour, 0.02 * cv2.arcLength(contour, True), True)
            if len(approx) == 3:
                cnt = np.squeeze(approx)
                position, angle =  find_pos_angle(cnt)
                self._position = np.array([position[0], position[1], angle])
                # # Draw a bounding box around the detected triangle
                # x, y, w, h = cv2.boundingRect(contour)
                # cv2.drawContours(frame, [contour], 0, (0, 255, 0), 2)
                # cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
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
            self.__intermediary_tracker = 0

    def is_on_objective(self):
        return self.__on_objective

    def show_image(self, transformed: bool = True, draw_path: bool = True, draw_vertices: bool = True):
        if self.__get_most_recent_image():
            if draw_path:
                self.calculate_global_navigation()
                draw_path_on_camera(self.__new_perspective_image, self.__shortest_path, self.__obstacle_vertices,
                                    self._position[0:2], self._position[2])
            if draw_vertices:
                cv2.polylines(self.__new_perspective_image, [self.__qr_vertices.astype(int)], isClosed=True,
                            color=(255, 0, 0), thickness=0)

            if transformed:
                if self.__new_perspective_image is not None:
                    cv2.imshow(TRANSFORMED_IMAGE_NAME, self.__new_perspective_image)
                else:
                    cv2.imshow(NORMAL_IMAGE_NAME, self.__image)
            else:
                cv2.imshow(NORMAL_IMAGE_NAME, self.__image)
                
        else:
            print("no image to show")
            return False

    def get_next_position(self):
        if len(self.__shortest_path) <= self.__intermediary_tracker:
            self.__on_objective = True
        if self.__shortest_path is not None:
            vg_point_next_pos = self.__shortest_path[1]
            next_pos_to_go = np.array([vg_point_next_pos.x, vg_point_next_pos.y])
            self.__intermediary_tracker += 1
            if len(self.__shortest_path) == 2:
                next_angle = np.arctan2((next_pos_to_go[1] - self._position[1]), (next_pos_to_go[0] - self._position[0]))
                next_angle = convert_angle(next_angle)
            else:                        
                vg_point_next_next_pos = self.__shortest_path[2]
                next_next_pos_to_go = np.array([vg_point_next_next_pos.x, vg_point_next_next_pos.y])
                next_angle = np.arctan2(next_next_pos_to_go[1] - next_pos_to_go[1], next_next_pos_to_go[0] - next_pos_to_go[0])
                next_angle = convert_angle(next_angle)
            return np.array([next_pos_to_go[0], next_pos_to_go[1], next_angle])
        else:
            return None

    def __del__(self):
        cv2.destroyWindow(NORMAL_IMAGE_NAME)
        cv2.destroyWindow(TRANSFORMED_IMAGE_NAME)
        self.__video_stream.release()