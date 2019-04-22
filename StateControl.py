from Navigation import Navigation as Nav
from enum import Enum
import time
import cv2 as cv


class LostTargetException(Exception):
    pass


class PrmState(Enum):
    TRAVEL_MINING = 0
    MINING = 1
    TRAVEL_GOAL = 2
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
        self.primary_state = PrmState.TRAVEL_MINING
        self.secondary_state = SecState.SEARCH
        # global variables
        self.last_seen_time = -1  # default to negative value so that the first run always works
        self.goal = 0  # index for the current goal type to look for
        # face cascades
        base_path = "/Users/coryjohns/Desktop/School/CSCI/csci442_a2/venv/lib/python3.7/site-packages/cv2/"
        self.face_cascade = cv.CascadeClassifier(base_path + 'data/haarcascade_frontalface_default.xml')

        # adjustable parameters
        # ratio of the current face distance and the standard distance, i.e current/standard, that is acceptable
        # values less then 1 occur when target is far away
        self.distance_ratio = 0.9
        self.face_width_standard = 140  # this value is for ~1 meter from the laptop camera

    def main_loop_step(self, frame):
        """
        Performs one step of the current state and handles transitions between states
        State Order for one cycle:
            0) find mining area = find_state + target_mining_area
            1) travel to mining area = obstacle_avoidance_state + target_mining_area
            2) find human = find_state + target_human
            3) travel to human = traveling_state + target_human -> ask for ice once there
            4) identify ice -> grab ice once correct
            5) find goal area = find_state + target_goal_area
            6) return to start = obstacle_avoidance_state + target_goal_area
            7) find goal area = find_state + target_goal_area for current ice target
            8) travel to goal area = traveling_state + target_goal_area
            9) drop ice in correct goal area
        :param frame: the current frame of the camera
        :return: True on a successful completion of a cycle
        """
        if frame is not None:

            # >>>> MOVING States >>>>
            if self.secondary_state == SecState.MOVING:
                if self.primary_state == PrmState.TRAVEL_MINING:
                    # 1) travel to mining area = obstacle_avoidance_state + target_mining_area
                    if self.debug: print("STATE = 1, travel to mining area")
                    try:
                        if self.obstacle_avoidance_state(frame, self.target_mining_area):
                            # reached mining area since function returned True
                            # TODO declare that mining area is reached
                            if self.debug: print("STATE = 1, mining area reached")
                            # set to next state
                            self.primary_state = PrmState.MINING
                            self.secondary_state = SecState.SEARCH
                    except LostTargetException or TypeError:
                        # revert back to search state
                        self.secondary_state = SecState.SEARCH

                elif self.primary_state == PrmState.MINING:
                    # 3) travel to human = traveling_state + target_human -> ask for ice once there
                    if self.debug: print("STATE = 3, travel to human")
                    try:
                        if self.traveling_state(frame, self.target_human):
                            # reached human since function returned True
                            # TODO ask for ice
                            if self.debug: print("STATE = 3, asking for ice")
                            # set to next state
                            self.secondary_state = SecState.ACTING
                    except LostTargetException or TypeError:
                        # revert back to search state
                        self.secondary_state = SecState.SEARCH

                elif self.primary_state == PrmState.TRAVEL_GOAL:
                    # 6) return to start = obstacle_avoidance_state + target_goal_area
                    if self.debug: print("STATE = 6, return to start")
                    try:
                        if self.obstacle_avoidance_state(frame, self.target_goal_area):
                            # reached start area since function returned True
                            # TODO declare goal area is reached
                            if self.debug: print("STATE = 6, goal area reached")
                            # set to next state
                            self.primary_state = PrmState.GOAL
                            self.secondary_state = SecState.SEARCH
                    except LostTargetException or TypeError:
                        # revert back to search state
                        self.secondary_state = SecState.SEARCH

                else:  # PimState == GOAL
                    # 8) travel to goal area = traveling_state + target_goal_area
                    if self.debug: print("STATE = 8, travel to goal area")
                    try:
                        if self.traveling_state(frame, self.target_goal_area):
                            # reached goal area since function returned True
                            if self.debug: print("STATE = 8, goal area reached")
                            # set to next state
                            self.secondary_state = SecState.ACTING
                    except LostTargetException or TypeError:
                        # revert back to search state
                        self.secondary_state = SecState.SEARCH

            # >>>> SEARCH States >>>>
            elif self.secondary_state == SecState.SEARCH:
                if self.primary_state == PrmState.TRAVEL_MINING:
                    # 0) find mining area = find_state + target_mining_area
                    if self.debug: print("STATE = 0, find mining area")
                    if self.find_state(frame, self.target_mining_area):
                        # detected mining area since function returned True
                        if self.debug: print("STATE = 0, mining area found")
                        # set to next state
                        self.secondary_state = SecState.MOVING

                elif self.primary_state == PrmState.MINING:
                    # 2) find human = find_state + target_human
                    if self.debug: print("STATE = 2, find human")
                    if self.find_state(frame, self.target_human):
                        # detected human since function returned True
                        if self.debug: print("STATE = 2, human found")
                        # set to next state
                        self.secondary_state = SecState.MOVING

                elif self.primary_state == PrmState.TRAVEL_GOAL:
                    # 5) find start area = find_state + target_goal_area
                    if self.debug: print("STATE = 5, find start area")
                    if self.find_state(frame, self.target_goal_area):
                        # detected start area since function returned True
                        if self.debug: print("STATE = 5, start area found")
                        # set to next state
                        self.secondary_state = SecState.MOVING

                else:  # PimState == GOAL
                    # 7) find goal area = find_state + target_goal_area for current ice target
                    if self.debug: print("STATE = 7, find goal area")
                    if self.find_state(frame, self.target_goal_area):
                        # detected goal area since function returned True
                        if self.debug: print("STATE = 7, goal area found")
                        # set to next state
                        self.secondary_state = SecState.MOVING

            # >>>> ACTING States >>>>
            else:  # SecState == ACTING
                if self.primary_state == PrmState.MINING:
                    # 4) identify ice -> grab ice once correct
                    if self.debug: print("STATE = 4, grabbing ice")
                    if self.grab_ice(frame, self.goal):
                        # grab ice success
                        if self.debug: print("STATE = 4, grab success")
                        # set to next state
                        self.primary_state = PrmState.TRAVEL_GOAL
                        self.secondary_state = SecState.SEARCH
                    else:
                        # TODO ask for correct ice
                        if self.debug: print("STATE = 4, grab failure")

                elif self.primary_state == PrmState.GOAL:
                    # 9) drop ice in correct goal area
                    if self.debug: print("STATE = 9, dropping ice")
                    if self.drop_ice(frame, self.goal):
                        # drop ice success
                        if self.debug: print("STATE = 9, drop success")
                        # set to default state
                        self.primary_state = PrmState.TRAVEL_MINING
                        self.secondary_state = SecState.SEARCH
                        # cycle complete
                        return True
                    else:
                        # drop ice failure
                        if self.debug: print("STATE = 9, drop failure")
                else:
                    raise Warning("TRAVEL_* primary states not defined for ACTING secondary state")
        # cycle incomplete
        return False

    def obstacle_avoidance_state(self, frame, targeting_function, retargeting_timeout=0.5, suppress_exception=False):
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
            if dis > self.distance_ratio:
                return True
        except LostTargetException or TypeError:
            if time.process_time() - self.last_seen_time > retargeting_timeout:
                # it has been to long since we last saw the target, will have to transition back into search state
                if suppress_exception:
                    return None
                else:
                    raise LostTargetException("Target lost after timeout")
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
            if distance < self.distance_ratio:
                # get function for moving or rotating
                move_function = self.navigation_obj.get_needed_action(location[0] - frame.shape[1] // 2)
                # do action
                move_function()
                return False
            else:
                return True
        # handle losing the target
        except LostTargetException or TypeError:
            if time.process_time() - self.last_seen_time > retargeting_timeout:
                # it has been to long since we last saw the target, will have to transition back into search state
                if suppress_exception:
                    return None
                else:
                    raise LostTargetException("Target lost after timeout")

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
        :return: est. distance to target and its location (x, y) on the screen,
        raises a LostTargetException if the target was not found
        """
        # TODO debug temporarily disabled
        if False and self.debug:
            tmp = input("at/not/lost:")
            if tmp.__contains__("at"):
                # at target
                return [5, [frame.shape[0] // 2, frame.shape[1] // 2]]
            elif tmp.__contains__("not"):
                # not at target
                return [0, [frame.shape[0] // 2, frame.shape[1] // 2]]
            else:
                # lost target
                raise LostTargetException("TESTING, target lost in target_human")
        else:
            # look for a face in the frame
            gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
            faces = self.face_cascade.detectMultiScale(gray, 1.8, 5)
            if len(faces) > 0:
                # identify face to use
                (x, y, w, h) = faces[0]
                if self.debug: cv.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
                # get est. distance to face
                dis_ratio = w / self.face_width_standard
                # get face location
                face_loc = [x + w//2, y + h//2]
                return dis_ratio, face_loc
            else:
                # face was not found
                if suppress_exception:
                    return None
                else:
                    raise LostTargetException("Face not found")

    def target_mining_area(self, frame, suppress_exception=False):
        """
        Finds if the mining area identifier is on the screen, and if so est. the distance
        and returns the location of the mining area identifier on the screen.
        :param frame: The current camera frame
        :param suppress_exception: if True, the exception will not be raised and the function will return None instead
        :return: est. distance to target and its location (x, y) on the screen,
        raises a LostTargetException if the target was not found
        """
        if self.debug:
            tmp = input("at/not/lost:")
            if tmp.__contains__("at"):
                # at target
                return [5, [frame.shape[0] // 2, frame.shape[1] // 2]]
            elif tmp.__contains__("not"):
                # not at target
                return [0, [frame.shape[0] // 2, frame.shape[1] // 2]]
            else:
                # lost target
                raise LostTargetException("TESTING, target lost in target_mining_area")
        else:
            # TODO add non-debug function body
            pass

    def target_goal_area(self, frame,  goal_type, suppress_exception=False):
        """
        Finds if the goal area is on the screen, and if so est. the distance
        and returns the location of the goal area on the screen. If the goal area is close enough, will return the
        location of the specific goal area instead.
        :param frame: The current camera frame
        :param goal_type: the type of goal to search for, e.i. small, medium, large
        :param suppress_exception: if True, the exception will not be raised and the function will return None instead
        :return: est. distance to target and its location (x, y) on the screen,
        raises a LostTargetException if the target was not found
        """
        if self.debug:
            tmp = input("at/not/lost:")
            if tmp.__contains__("at"):
                # at target
                return [5, [frame.shape[0] // 2, frame.shape[1] // 2]]
            elif tmp.__contains__("not"):
                # not at target
                return [0, [frame.shape[0] // 2, frame.shape[1] // 2]]
            else:
                # lost target
                raise LostTargetException("TESTING, target lost in target_goal_area")
        else:
            # TODO add non-debug function body
            pass

    def grab_ice(self, frame, goal_type):
        """
        Detects if the relevant ice is in the gripers, and closes them if it is.
        :param frame: the current frame of the camera
        :param goal_type: the type of goal to search for, e.i. small, medium, large
        :return: True if ice was acquired, False otherwise
        """
        if self.debug:
            tmp = input("T/F:").__contains__("T")
            if tmp:
                # function success
                return True
            else:
                # function failure
                return False
        else:
            # TODO add non-debug function body
            pass

    def drop_ice(self, frame, goal_type):
        """
        Continues process to drop the ice in the relevant goal.
        :return: True if ice was dropped, False otherwise
        """
        if self.debug:
            tmp = input("T/F:").__contains__("T")
            if tmp:
                # function success
                return True
            else:
                # function failure
                return False
        else:
            # TODO add non-debug function body
            pass

