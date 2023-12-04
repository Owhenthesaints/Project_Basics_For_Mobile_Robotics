import cv2
from global_navigation.GlobalNav2 import GlobalNav2

# CAMERA INITIALISATION
global_navigation = GlobalNav2()
#print("done")
while True:
    while not global_navigation.find_thymio():
        pass
    
    # plt.
    global_navigation.show_image(True, True, False)
    print(global_navigation._position)
    key = cv2.waitKey(20)
    if key == 27: # exit on ESC
        break
