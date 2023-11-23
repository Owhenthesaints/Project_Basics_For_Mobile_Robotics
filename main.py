import numpy as np

import dependencies.constants_robot as constants_robot
from thymio_movement.ThymioRobot import ThymioRobot


def main():
    # initialise vision

    # initialisation of the thymio
    little_thymio = ThymioRobot()
    while little_thymio.is_alive:
        if np.any(little_thymio.get_sensors() > constants_robot.LOCAL_NAV_SWITCH):
            little_thymio.local_nav()

        # detect position of thymio visually
        # apply kallman
        # set the new position as argument of set_new_position
        little_thymio.set_new_position()
        if little_thymio.on_objective:
            # then give it the next node/objective
            little_thymio.set_new_goal()
        if kidnapped:
            # here do steps to find new_goal
            little_thymio.set_new_goal()

        # just to apply the astolfi command
        little_thymio.apply_motor_command()

        # here make final condition if thymio ends up on the final point
        if ...:
            little_thymio.kill()


if __name__ == "__main__":
    main()
