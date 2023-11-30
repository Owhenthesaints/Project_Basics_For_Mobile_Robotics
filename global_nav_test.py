
from global_navigation.GlobalNav2 import GlobalNav2

# CAMERA INITIALISATION
global_navigation = GlobalNav2()
#print("done")
# while global_navigation.get_robot_pos_and_angle() is None:
#     print("not found")
# global_navigation.calculate_global_navigation()
while True:
    global_navigation.show_image(False, False, False)