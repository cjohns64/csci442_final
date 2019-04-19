import cv2 as cv
import navigation as nav
from enum import Enum
import time


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
        self.debug = debug
        # initialize with base state
        self.primary_state = PrmState.OBS_AVOID
        self.secondary_state = SecState.SEARCH
        # global variables
        self.last_seen_time = -1  # default to negative value so that the first run always works

        # adjustable parameters
        self.distance_value = 10

    def obstacle_avoidance_state(self, frame, targeting_function, retargeting_timeout=2):
        """
        Continues to navigate through the obstacles, also takes into account if the target is seen.
        :param frame: the current frame of the camera
        :param targeting_function: function used to identify distance and direction to target,
        returns est. distance to target and its location on the screen,
        raises a LostTargetException if the target is lost
        :param retargeting_timeout: time delay between researching for the target
        :return: True if target is reached, False if it has not,
        raises a LostTargetException if the target is lost after the retargeting_timeout
        """
        try:
            # try to find the target
            dis, loc = targeting_function(frame)
            if dis < self.distance_value:
                return True
        except LostTargetException:
            if self.last_seen_time - time.process_time() > retargeting_timeout:
                # it has been to long since we last saw the target, will have to transition back into search state
                raise LostTargetException("Target lost after {} seconds".format(retargeting_timeout))
        # continue along available path
        # get path target location on the screen
        nav_x, nav_y = self.navigation_obj.get_path(frame)
        # get the movement function we need
        move_function = self.navigation_obj.get_needed_action(nav_x - frame.shape[1] // 2)
        # do action
        move_function()
        return False

    def traveling_state(self, frame, targeting_function):
        """
        Continues to travel to target, must be given a targeting function
        :param frame: the current camera frame
        :param targeting_function: function used to identify distance and direction to target,
        returns est. distance to target and its location on the screen,
        raises a LostTargetException if the target is lost
        :return: True if target is reached, False if it has not,
        raises a LostTargetException if the target is lost
        """
        # find the object, LostTargetException raised if targeting_function fails to find the target
        distance, location = targeting_function(frame)
        if distance > self.distance_value:
            # get function for moving or rotating
            move_function = self.navigation_obj.get_needed_action(location[0] - frame.shape[1] // 2)
            # do action
            move_function()
            return False
        else:
            return True

    def find_state(self, frame, targeting_function):
        """
        Continues to search for the target, given by the targeting_function.
        Assumes the head is already in the correct orientation
        :param frame: the current camera frame
        :param targeting_function: function used to identify distance and direction to target,
        returns est. distance to target and its location on the screen,
        raises a LostTargetException if the target is lost
        :return: True if the target is found, False if it has not
        """
        # search for target, if it is not found rotate right
        try:
            dis, loc = targeting_function(frame)
            return True
        except LostTargetException:
            # target not found, rotate right
            self.navigation_obj.rotate_right()
            return False

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
        if frame is not None:
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
