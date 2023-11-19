import numpy as np


#Threshold to switch states
# values need tuning
obstThrH = 2000
obstThrL = 1000

# obstacle avoidance: ANN weights
w = np.array(
[[50, 30, -20, -30, -50, 30, -10, 8, 0],
[-50, -30, -20, 30, 50, -10, 30, 0, 8]]) 



# Scale factors that divide sensor inputs and memory inputs
sensor_scale = 800
memory_scale = 20

# here i want to use a state flag to switch the thymio between local navigation and glabal navigation
def update_state(state, obst):
    if state == 1:
        # switch from goal tracking to obst avoidance if obstacle detected
        if (obst > obstThrH).any():
            state = 2
    elif state == 2:
        if (obst < obstThrL).all():
            state = 1
    return state

def local_nav(prox_horizontal : list, y_hist : list) -> np.ndarray:

    x = np.zeros(9, dtype=np.int64)

    # Memory
    x[7:9] = np.array(y_hist) // memory_scale

    # Get and scale inputs
    x[:7] = np.array(prox_horizontal) // sensor_scale

    # Compute outputs of neurons and set motor powers
    # y_ctrl = [left_speed, right_speed]
    y_ctrl = w @ x

    return y_ctrl