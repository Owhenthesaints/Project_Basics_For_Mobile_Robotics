import cv2
from global_navigation.GlobalNav2 import GlobalNav2

# CAMERA INITIALISATION
global_navigation = GlobalNav2()
#print("done")
#while global_navigation.get_robot_pos_and_angle() is None:
#     print("not found")
#global_navigation.calculate_global_navigation()
while True:
    print(global_navigation.get_robot_pos_and_angle())
    global_navigation.show_image(True, True, False)
    key = cv2.waitKey(20)
    if key == 27: # exit on ESC
        break
