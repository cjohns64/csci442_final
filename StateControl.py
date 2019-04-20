from Navigation import Navigation as Nav
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


class StateController:
    """
    Controls the states of the robot, given the current frame
    """

    def __init__(self, debug=False):
        # set up objects
        self.navigation_obj = Nav(display=True)
        self.debug = debug
        # initialize with base state
        self.primary_state = PrmState.OBS_AVOID
        self.secondary_state = SecState.SEARCH
        # global variables
        self.last_seen_time = -1  # default to negative value so that the first run always works

        # adjustable parameters
        self.distance_value = 10

    def main_loop_step(self, frame):
        """
        Performs one step of the current state and handles transitions between states
        :param frame:
        :return:
        """
        if frame is not None:
            # TODO transition between states
            self.obstacle_avoidance_state(frame, self.target_mining_area)

    def obstacle_avoidance_state(self, frame, targeting_function, retargeting_timeout=2, suppress_exception=False):
        """
        Continues to navigate through the obstacles, also takes into account if the target is seen.
        :param frame: the current frame of the camera
        :param targeting_function: function used to identify distance and direction to target,
        returns est. distance to target and its location on the screen,
        raises a LostTargetException if the target is lost
        :param retargeting_timeout: time delay between researching for the target
        :param suppress_exception: if True, the targeting_function will not use exceptions
        :return: True if target is reached, False if it has not,
        raises a LostTargetException if the target is lost after the retargeting_timeout
        """
        try:
            # try to find the target,
            dis, loc = targeting_function(frame, suppress_exception)
            # update the last seen time with the current time
            self.last_seen_time = time.process_time()
            if dis < self.distance_value:
                return True
        except LostTargetException or TypeError:
            if self.last_seen_time - time.process_time() > retargeting_timeout:
                # it has been to long since we last saw the target, will have to transition back into search state
                if suppress_exception:
                    return None
                else:
                    raise LostTargetException("Target lost after {} seconds".format(retargeting_timeout))
        # continue along available path
        # get path target location on the screen
        nav_x, nav_y = self.navigation_obj.get_path(frame)
        # get the movement function we need
        move_function = self.navigation_obj.get_needed_action(nav_x - frame.shape[1] // 2)
        # do action
        move_function()
        return False

    def traveling_state(self, frame, targeting_function, retargeting_timeout=0.5, suppress_exception=False):
        """
        Continues to travel to target, must be given a targeting function
        :param frame: the current camera frame
        :param targeting_function: function used to identify distance and direction to target,
        returns est. distance to target and its location on the screen,
        raises a LostTargetException if the target is lost
        :param retargeting_timeout: time delay between researching for the target
        :param suppress_exception: if True, the exception will not be raised and the function will return None instead
        :return: True if target is reached, False if it has not,
        raises a LostTargetException if the target is lost
        """
        try:
            # find the object, LostTargetException raised if targeting_function fails to find the target
            distance, location = targeting_function(frame, suppress_exception)
            # update the last seen time with the current time
            self.last_seen_time = time.process_time()
            # check distance to target
            if distance > self.distance_value:
                # get function for moving or rotating
                move_function = self.navigation_obj.get_needed_action(location[0] - frame.shape[1] // 2)
                # do action
                move_function()
                return False
            else:
                return True
        # handle losing the target
        except LostTargetException or TypeError:
            if self.last_seen_time - time.process_time() > retargeting_timeout:
                # it has been to long since we last saw the target, will have to transition back into search state
                if suppress_exception:
                    return None
                else:
                    raise LostTargetException("Target lost after {} seconds".format(retargeting_timeout))

    def find_state(self, frame, targeting_function, suppress_exception=False):
        """
        Continues to search for the target, given by the targeting_function.
        Assumes the head is already in the correct orientation
        :param frame: the current camera frame
        :param targeting_function: function used to identify distance and direction to target,
        returns est. distance to target and its location on the screen,
        raises a LostTargetException if the target is lost
        :param suppress_exception: if True, the targeting_function will not use exceptions
        :return: True if the target is found, False if it has not
        """
        # search for target, if it is not found rotate right
        try:
            dis, loc = targeting_function(frame, suppress_exception)
            return True
        # catch event that we lost the target, if exceptions are suppressed TypeError will catch the None type return
        except LostTargetException or TypeError:
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
