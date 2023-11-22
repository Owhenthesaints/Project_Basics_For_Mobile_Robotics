import numpy as np
import dependencies.constants_robot as cst



# here i want to use a state flag to switch the thymio between local navigation and glabal navigation
def update_state(state, obst):
    if state == 1:
        # switch from goal tracking to obst avoidance if obstacle detected
        if (obst > cst.OBSTTHRH).any():
            state = 2
    elif state == 2:
        if (obst < cst.OBSTTHRL).all():
            state = 1
    return state

def local_nav(prox_horizontal : list, y_hist : list) -> np.ndarray:

    x = np.zeros(9, dtype=np.int64)

    # Memory
    x[7:9] = np.array(y_hist) // cst.MEMORY_SCALE

    # Get and scale inputs
    x[:7] = np.array(prox_horizontal) // cst.SENSOR_SCALE

    # Compute outputs of neurons and set motor powers
    # y_ctrl = [left_speed, right_speed]
    y_ctrl = cst.w @ x

    return y_ctrl