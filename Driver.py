import cv2 as cv
import navigation as nav
from enum import Enum


class LostTargetException(Exception):
    pass


class PrmState(Enum):
    OBS_AVOID = 0
    MINING = 1
    RETURNING = 2
    GOAL = 3


class SecState(Enum):
    SEARCH = 0
    MOVING = 1
    ACTING = 3


class Driver:
    # TODO implement state list
    # find mining area = find_state + target_mining_area

    # travel to mining area = obstacle_avoidance_state + target_mining_area

    # find human = find_state + target_human

    # travel to human = traveling_state + target_human -> ask for ice once there

    # identify ice -> grab ice once correct

    # find start area = find_state + target_start_area

    # return to start = obstacle_avoidance_state + target_start_area

    # find goal area = find_state + target_goal_area for current ice target

    # travel to goal area = traveling_state + target_goal_area -> drop ice once there

    def __init__(self, debug=False):
        # set up objects
        self.navigation_obj = nav.PathDetection(display=True)
        self.move_function = None
        self.debug = debug
        # initialize with base state
        self.primary_state = PrmState.OBS_AVOID
        self.secondary_state = SecState.SEARCH

    def obstacle_avoidance_state(self, frame):
        """
        State for navigating through the obstacles
        :param frame: the current frame of the camera
        :return:
        """
        if self.debug: print("STATE: obstacle avoidance")
        if frame is not None:
            # get path target location on the screen
            nav_x, nav_y = self.navigation_obj.get_path(frame)
            # get the movement function we need
            self.move_function = self.navigation_obj.get_rotation(nav_x - frame.shape[1] // 2)
            # TODO use the movement function
            pass

    def traveling_state(self, targeting_function):
        """
        Continues to travel to target, must be given a targeting function
        :param targeting_function: function used to identify distance and direction to target,
        returns est. distance to target and its location on the screen,
        raises a LostTargetException if the target is lost
        :return: True if target is reached, False if it has not,
        raises a LostTargetException if the target is lost
        """
        pass

    def find_state(self, targeting_function):
        """
        Continues to search for the target, given by the targeting_function
        :param targeting_function: function used to identify distance and direction to target,
        returns est. distance to target and its location on the screen,
        raises a LostTargetException if the target is lost
        :return: True if the target is found, False if it has not
        """
        pass

    def target_human(self, frame, suppress_exception=False):
        """
        Finds if the human is on the screen, and if so est. the distance
        and returns the location of the face on the screen.
        :param frame: The current camera frame
        :param suppress_exception: if True, the exception will not be raised and the function will return None instead
        :return: est. distance to target and its location on the screen,
        raises a LostTargetException if the target was not found
        """
        pass

    def target_mining_area(self, frame, suppress_exception=False):
        """
        Finds if the mining area identifier is on the screen, and if so est. the distance
        and returns the location of the mining area identifier on the screen.
        :param frame: The current camera frame
        :param suppress_exception: if True, the exception will not be raised and the function will return None instead
        :return: est. distance to target and its location on the screen,
        raises a LostTargetException if the target was not found
        """
        pass

    def target_start_area(self, frame, suppress_exception=False):
        """
        Finds if the start area identifier is on the screen, and if so est. the distance
        and returns the location of the start area identifier on the screen.
        :param frame: The current camera frame
        :param suppress_exception: if True, the exception will not be raised and the function will return None instead
        :return: est. distance to target and its location on the screen,
        raises a LostTargetException if the target was not found
        """
        pass

    def target_goal_area(self, frame,  goal_type, suppress_exception=False):
        """
        Finds if the goal area is on the screen, and if so est. the distance
        and returns the location of the goal area on the screen. If the goal area is close enough, will return the
        location of the specific goal area instead.
        :param frame: The current camera frame
        :param goal_type: the type of goal to search for, e.i. small, medium, large
        :param suppress_exception: if True, the exception will not be raised and the function will return None instead
        :return: est. distance to target and its location on the screen,
        raises a LostTargetException if the target was not found
        """
        pass

    def main_loop_step(self, frame):
        # TODO transition between states
        self.obstacle_avoidance_state(frame)


def laptop_cam_loop(obj):
    # set up video capture
    cap = cv.VideoCapture(0)
    cv.namedWindow("Video")

    # set screen size
    cap.set(cv.CAP_PROP_FRAME_WIDTH, 400)
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, 300)
    while True:
        # get video info
        status, frame = cap.read()

        # run one frame of the main operating loop
        obj.main_loop_step(frame)

        cv.imshow("Video", frame)

        k = cv.waitKey(1)
        # this is the "esc" key
        if k == 27:
            break
    cv.destroyAllWindows()


# start robot state object
robot = Driver(debug=True)
# run w/ laptop camera
laptop_cam_loop(robot)
