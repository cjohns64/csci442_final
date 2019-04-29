from global_settings import *
from Navigation import Navigation as Nav
from enum import Enum
from TimeDelay import TimeDelay as delay
import time
import cv2 as cv
import numpy as np
if use_phone: from client import ClientSocket


if not laptop:
    IP = '10.200.46.186'
    PORT = 5010
    face = None
    # set up client and face searching
    if use_phone: client = ClientSocket(IP, PORT)


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
    ACTING = 2


class Location(Enum):
    MINING_AREA = 0
    ROCK_AREA = 1
    GOAL_AREA = 2


class StateController:
    """
    Controls the states of the robot, given the current frame
    """

    def __init__(self, debug=False):
        # set up objects
        self.navigation_obj = Nav(display=True, debug=debug)
        self.debug = debug
        # initialize with base state
        self.primary_state = PrmState.TRAVEL_MINING
        self.secondary_state = SecState.SEARCH
        self.current_loc = Location.GOAL_AREA
        self.last_loc = None
        self.navigation_obj.tilt_head_to_search()
        self.blur_frame = True

        # global variables
        self.goal = 1  # index for the current goal type to look for (green == 0, pink == 1)
        self.start = 1
        self.face_cascade = cv.CascadeClassifier('haarcascade_frontalface_default.xml')

        # Time Delay variables
        self.target_timeout = delay(1.6)  # time delay between reverting back to searching state
        self.rotate_delay = delay(0.4)  # pause between rotation and looking while searching for a face
        # TODO self.face_timeout = delay(1)  # time to wait before reverting back to searching for a face state
        self.zone_change_delay = delay(0.6)  # delay before another zone change is allowed

        # adjustable parameters
        self.color_tolerance = np.array([10, 15, 200])  # np.array([20, 20, 250])  # HSV, accept most values
        # ratio of the current face distance and the standard distance, i.e current/standard, that is acceptable
        # values less then 1 occur when target is far away
        self.distance_ratio = 0.9
        # standard distances for targeting functions
        # 1 distance value is recorded at the optimum distance
        # and the ratio of the current sensor value and this distance
        # will be compared to the distance_ratio to determine if we have reached the target or not
        self.face_width_standard = 50  # this value is for ~1 meter from the laptop camera
        self.mining_area_standard = 50
        self.goal_small_standard = 70
        self.goal_large_standard = 120
        # color standard values based off of sampling
        self.pink_standard = [160, 130, 240]  # BGR [190, 125, 250]
        self.green_standard = [50, 170, 150]  # BGR [25, 200, 110]
        self.orange_line_standard = [20, 50, 250]  # BGR [160, 215, 240]
        self.mining_indicator_standard = self.green_standard
        if use_phone: client.sendData("You are connected")

    @staticmethod
    def exit():
        if use_phone: client.killSocket()

    def zero_motors(self):
        self.navigation_obj.zero_motors()

    def get_frame_blur(self):
        return self.blur_frame

    def get_state_index(self):
        s = self.secondary_state.value
        p = self.primary_state.value
        if s == 0:
            if p == 0:
                return 0
            elif p == 1:
                return 2
            elif p == 2:
                return 5
            else:
                return 7
        elif s == 1:
            if p == 0:
                return 1
            elif p == 1:
                return 3
            elif p == 2:
                return 6
            else:
                return 8
        else:
            if p == 1:
                return 4
            elif p == 3:
                return 9
            else:
                print("get state error")
                pass

    def is_debug_ignore_state(self):
        state = self.get_state_index()
        for i in no_debug_states:
            if i == state:
                return True
        return False

    def calibrate_face_detection(self, frame, one, two):
        # look for a face in the frame
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        faces = self.face_cascade.detectMultiScale(gray, one, two)
        if len(faces) > 0:
            print("faces", len(faces))
            # identify face to use
            (x, y, w, h) = faces[0]
            if self.debug: cv.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
            # get est. distance to face
            dis_ratio = w / self.face_width_standard
            # get face location
            face_loc = [x + w // 2, y + h // 2]
            return dis_ratio, face_loc

    def main_loop_step(self, frame):
        """
        Performs one step of the current state and handles transitions between states
        State Order for one cycle:
            0) find mining area = find_state + target_mining_area
            1) travel to mining area = obstacle_avoidance_state + target_mining_area
            2) find human = find_state + target_human
            3) travel to human = traveling_state + target_human -> ask for ice once there
            4) identify ice -> grab ice once correct
            5) find goal area = find_state + target_start_area
            6) return to start = obstacle_avoidance_state + target_start_area
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
                        if self.travel_or_avoid(frame, self.target_mining_area):
                            # reached mining area since function returned True
                            # declare that mining area is reached
                            if not laptop and use_phone: client.sendData("The mining area has been reached")
                            print("STATE = 1, mining area reached")
                            # set to next state
                            self.primary_state = PrmState.MINING
                            self.transition_to_search_state(True)
                    except LostTargetException or TypeError:
                        # revert back to search state
                        self.transition_to_search_state()

                elif self.primary_state == PrmState.MINING:
                    # 3) travel to human = traveling_state + target_human -> ask for ice once there
                    print("STATE = 3, travel to human")
                    self.blur_frame = False  # turn off frame blurring for face detection
                    try:
                        if self.travel_or_avoid(frame, self.target_human):
                            # reached human since function returned True
                            print("STATE = 3, asking for ice")
                            self.blur_frame = True
                            # ask for ice.
                            if not laptop and use_phone: client.sendData("May I please have some ice")
                            # set to next state
                            self.transition_to_acting_state()
                    except LostTargetException or TypeError:
                        # revert back to search state
                        self.transition_to_search_state(True)

                elif self.primary_state == PrmState.TRAVEL_GOAL:
                    # 6) return to start = obstacle_avoidance_state + target_start_area
                    print("STATE = 6, return to start")
                    try:
                        if self.travel_or_avoid(frame, self.target_start_area):
                            # reached start area since function returned True
                            # declare goal area is reached
                            if not laptop and use_phone:
                                client.sendData("We have reached the goal area")
                                time.sleep(1.5)
                            print("STATE = 6, start area reached")
                            # set to next state
                            self.primary_state = PrmState.GOAL
                            self.transition_to_search_state()
                    except LostTargetException or TypeError:
                        # revert back to search state
                        self.transition_to_search_state()

                else:  # PimState == GOAL
                    # 8) travel to goal area = traveling_state + target_goal_area
                    print("STATE = 8, travel to goal area")
                    try:
                        if self.travel_or_avoid(frame, self.target_goal_area):
                            # reached goal area since function returned True
                            print("STATE = 8, goal area reached")
                            # set to next state
                            self.transition_to_acting_state()
                    except LostTargetException or TypeError:
                        # revert back to search state
                        self.transition_to_search_state()

            # >>>> SEARCH States >>>>
            elif self.secondary_state == SecState.SEARCH:
                if self.primary_state == PrmState.TRAVEL_MINING:
                    # 0) find mining area = find_state + target_mining_area
                    print("STATE = 0, find mining area")
                    if self.find_state(frame, self.target_mining_area):
                        # detected mining area since function returned True
                        print("STATE = 0, mining area found")
                        # set to next state
                        self.transition_to_move_state()

                elif self.primary_state == PrmState.MINING:
                    # 2) find human = find_state + target_human
                    print("STATE = 2, find human")
                    self.blur_frame = False  # turn off frame blurring for face detection
                    if self.find_state(frame, self.target_human):
                        # detected human since function returned True
                        print("STATE = 2, human found")
                        # set to next state
                        self.blur_frame = True
                        self.transition_to_move_state(True)

                elif self.primary_state == PrmState.TRAVEL_GOAL:
                    # 5) find start area = find_state + target_start_area
                    print("STATE = 5, find start area")
                    if self.find_state(frame, self.target_start_area):
                        # detected start area since function returned True
                        print("STATE = 5, start area found")
                        # set to next state
                        self.transition_to_move_state()

                else:  # PimState == GOAL
                    # 7) find goal area = find_state + target_goal_area for current ice target
                    print("STATE = 7, find goal area")
                    if self.find_state(frame, self.target_goal_area):
                        # detected goal area since function returned True
                        print("STATE = 7, goal area found")
                        # set to next state
                        self.transition_to_goal_state()

            # >>>> ACTING States >>>>
            else:  # SecState == ACTING
                if self.primary_state == PrmState.MINING:
                    # 4) identify ice -> grab ice once correct
                    print("STATE = 4, grabbing ice")
                    if self.grab_ice(frame):
                        # grab ice success
                        print("STATE = 4, grab success")
                        # set to next state
                        self.primary_state = PrmState.TRAVEL_GOAL
                        self.transition_to_search_state()
                    else:
                        print("STATE = 4, grab failure")

                elif self.primary_state == PrmState.GOAL:
                    # 9) drop ice in correct goal area
                    print("STATE = 9, dropping ice")
                    if self.drop_ice():
                        # drop ice success
                        print("STATE = 9, drop success")
                        # set to default state
                        self.primary_state = PrmState.TRAVEL_MINING
                        self.transition_to_search_state()
                        # cycle complete
                        return True
                    else:
                        # drop ice failure
                        print("STATE = 9, drop failure")
                else:
                    raise Warning("TRAVEL_* primary states not defined for ACTING secondary state")
        # cycle incomplete
        return False

    def transition_to_search_state(self, human=False):
        if human:
            self.navigation_obj.tilt_head_to_human()
        else:
            self.navigation_obj.tilt_head_to_search()
        self.navigation_obj.zero_wheels()
        self.secondary_state = SecState.SEARCH

    def transition_to_move_state(self, human=False):
        if human:
            self.navigation_obj.tilt_head_to_human()
        else:
            self.navigation_obj.tilt_head_to_search()
        self.navigation_obj.zero_wheels()
        self.secondary_state = SecState.MOVING

    def transition_to_acting_state(self):
        self.navigation_obj.tilt_head_to_move()
        self.navigation_obj.zero_wheels()
        self.secondary_state = SecState.ACTING

    def transition_to_goal_state(self):
        self.navigation_obj.tilt_head_to_move()
        self.navigation_obj.zero_wheels()
        self.secondary_state = SecState.MOVING

    def travel_or_avoid(self, frame, targeting_function, suppress_exception=False):
        """
        ether activates the obstacle_avoidance_state or the traveling_state depending on what the current zone is.
        :param frame: the current frame of the camera
        :param targeting_function: function used to identify distance and direction to target,
        returns est. distance to target and its location on the screen,
        raises a LostTargetException if the target is lost
        :param suppress_exception: if True, the targeting_function will not use exceptions
        :return: True if target is reached, False if it has not,
        raises a LostTargetException if the target is lost after the retargeting timeout
        """
        # TODO obstacle_avoidance_state disabled
        if False and self.current_loc == Location.ROCK_AREA:
            # only avoid obstacles in the rock area
            return self.obstacle_avoidance_state(frame, targeting_function, suppress_exception)
        else:
            return self.traveling_state(frame, targeting_function, suppress_exception)

    def obstacle_avoidance_state(self, frame, targeting_function, suppress_exception=False):
        """
        Continues to navigate through the obstacles, also takes into account if the target is seen.
        :param frame: the current frame of the camera
        :param targeting_function: function used to identify distance and direction to target,
        returns est. distance to target and its location on the screen,
        raises a LostTargetException if the target is lost
        :param suppress_exception: if True, the targeting_function will not use exceptions
        :return: True if target is reached, False if it has not,
        raises a LostTargetException if the target is lost after the retargeting timeout
        """
        try:
            # try to find the target,
            dis, loc = targeting_function(frame, suppress_exception)
            # update the last seen time with the current time
            self.target_timeout.update_time()
            if dis > self.distance_ratio:
                return True
        except LostTargetException or TypeError:
            if not self.target_timeout.check_time():
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

    def traveling_state(self, frame, targeting_function, suppress_exception=False):
        """
        Continues to travel to target, must be given a targeting function
        :param frame: the current camera frame
        :param targeting_function: function used to identify distance and direction to target,
        returns est. distance to target and its location on the screen,
        raises a LostTargetException if the target is lost
        :param suppress_exception: if True, the exception will not be raised and the function will return None instead
        :return: True if target is reached, False if it has not,
        raises a LostTargetException if the target is lost
        """
        try:
            # find the object, LostTargetException raised if targeting_function fails to find the target
            distance, location = targeting_function(frame, suppress_exception)
            # update the last seen time with the current time
            self.target_timeout.update_time()
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
            if not self.target_timeout.check_time():
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
            if not self.rotate_delay.check_time():
                # target not found, rotate right
                self.navigation_obj.burst_right()
                self.rotate_delay.update_time()
            return False

    def target_human(self, frame, suppress_exception=False):
        """
        Finds if the human is on the screen, and if so est. the distance
        and returns the location of the face on the screen.
        :param frame: The current camera frame
        :param suppress_exception: if True, the exception will not be raised and the function will return None instead
        :return: est. distance (as a ratio) to target and its location (x, y) on the screen,
        raises a LostTargetException if the target was not found
        """
        if self.debug and not self.is_debug_ignore_state():
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
            faces = self.face_cascade.detectMultiScale(gray, 1.1, 5)
            if len(faces) > 0:
                print("faces", len(faces))
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
        :return: est. distance (as a ratio) to target and its location (x, y) on the screen,
        raises a LostTargetException if the target was not found
        """
        if self.debug and not self.is_debug_ignore_state():
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
            try:
                try:
                    # detect if orange_line is in the lower end of the frame
                    h, w = frame.shape[:2]
                    if self.find_color_in_frame(frame[int(h * 0.9):h, int(w * 0.3):int(w * 0.6)],
                                                self.orange_line_standard, suppress_exception):
                        # line detected
                        if self.current_loc == Location.GOAL_AREA and not self.zone_change_delay.check_time():
                            self.current_loc = Location.ROCK_AREA
                            self.zone_change_delay.update_time()
                        elif self.current_loc == Location.ROCK_AREA and not self.zone_change_delay.check_time():
                            self.current_loc = Location.MINING_AREA
                            self.zone_change_delay.update_time()
                except LostTargetException or TypeError:
                    pass  # no line no location change

                # get the width and location for the mining area indicator
                width, height, center = self.find_color_in_frame(frame, self.mining_indicator_standard, suppress_exception)
                # return distance ratio, and location of target
                return height / self.mining_area_standard, center
            except TypeError:
                # failed to find target
                # TypeErrors only occur when suppress_exception==True and the function failed to find the color
                # so pass the bad news on, if suppress_exception==False the LostTargetException will continue up
                return None

    def target_goal_area(self, frame, suppress_exception=False):
        """
        Finds if the goal area is on the screen, and if so est. the distance
        and returns the location of the goal area on the screen. If the goal area is close enough, will return the
        location of the specific goal area instead.
        :param frame: The current camera frame
        :param suppress_exception: if True, the exception will not be raised and the function will return None instead
        :return: est. distance (as a ratio) to target and its location (x, y) on the screen,
        raises a LostTargetException if the target was not found
        """
        if self.debug and not self.is_debug_ignore_state():
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
            try:
                try:
                    # detect if orange_line is in the lower end of the frame
                    h, w = frame.shape[:2]
                    if self.find_color_in_frame(frame[int(h * 0.9):h, int(w*0.3):int(w*0.6)],
                                                self.orange_line_standard, suppress_exception):
                        # line detected
                        if self.current_loc == Location.MINING_AREA and not self.zone_change_delay.check_time():
                            self.current_loc = Location.ROCK_AREA
                            self.zone_change_delay.update_time()
                        elif self.current_loc == Location.ROCK_AREA and not self.zone_change_delay.check_time():
                            self.current_loc = Location.GOAL_AREA
                            self.zone_change_delay.update_time()
                except LostTargetException or TypeError:
                    pass  # no line no location change

                # get the width and location for the given color
                if self.goal == 0:
                    width, _, center = self.find_color_in_frame(frame, self.green_standard, suppress_exception)
                    if width < self.goal_large_standard / 10:
                        # throw out cases where the detection was too small
                        if suppress_exception:
                            return None
                        else:
                            raise LostTargetException("detected target is too small")
                    else:
                        # return distance ratio, and location of target
                        return width / self.goal_large_standard, center
                else:
                    width, _, center = self.find_color_in_frame(frame, self.pink_standard, suppress_exception)
                    if width < self.goal_small_standard / 10:
                        # throw out cases where the detection was too small
                        if suppress_exception:
                            return None
                        else:
                            raise LostTargetException("detected target is too small")
                    else:
                        # return distance ratio, and location of target
                        return width / self.goal_small_standard, center
            except TypeError:
                # failed to find target
                # TypeErrors only occur when suppress_exception==True and the function failed to find the color
                # so pass the bad news on, if suppress_exception==False the LostTargetException will continue up
                return None

    def target_start_area(self, frame, suppress_exception=False):
        """
        Finds if the goal area is on the screen, and if so est. the distance
        and returns the location of the goal area on the screen. If the goal area is close enough, will return the
        location of the specific goal area instead.
        :param frame: The current camera frame
        :param suppress_exception: if True, the exception will not be raised and the function will return None instead
        :return: est. distance (as a ratio) to target and its location (x, y) on the screen,
        raises a LostTargetException if the target was not found
        """
        if self.debug and not self.is_debug_ignore_state():
            tmp = input("at/not/lost:")
            if tmp.__contains__("at"):
                # at target
                return [5, [frame.shape[0] // 2, frame.shape[1] // 2]]
            elif tmp.__contains__("not"):
                # not at target
                return [0, [frame.shape[0] // 2, frame.shape[1] // 2]]
            else:
                # lost target
                raise LostTargetException("TESTING, target lost in target_start_area")
        else:
            try:
                try:
                    # detect if orange_line is in the lower end of the frame
                    h, w = frame.shape[:2]
                    if self.find_color_in_frame(frame[int(h * 0.9):h, int(w * 0.3):int(w * 0.6)],
                                                self.orange_line_standard, suppress_exception):
                        # line detected
                        if self.current_loc == Location.MINING_AREA and not self.zone_change_delay.check_time():
                            self.current_loc = Location.ROCK_AREA
                            self.zone_change_delay.update_time()
                        elif self.current_loc == Location.ROCK_AREA and not self.zone_change_delay.check_time():
                            self.current_loc = Location.GOAL_AREA
                            self.zone_change_delay.update_time()
                except LostTargetException or TypeError:
                    pass  # no line no location change

                # get the width and location for the given color
                if self.start == 0:
                    width, _, center = self.find_color_in_frame(frame, self.green_standard, suppress_exception)
                    if width < self.goal_large_standard / 10:
                        # throw out cases where the detection was too small
                        if suppress_exception:
                            return None
                        else:
                            raise LostTargetException("detected target is too small")
                    else:
                        # return distance ratio, and location of target
                        return width / self.goal_large_standard, center
                else:
                    width, _, center = self.find_color_in_frame(frame, self.pink_standard, suppress_exception)
                    if width < self.goal_small_standard / 10:
                        # throw out cases where the detection was too small
                        if suppress_exception:
                            return None
                        else:
                            raise LostTargetException("detected target is too small")
                    else:
                        # return distance ratio, and location of target
                        return width / self.goal_small_standard, center
            except TypeError:
                # failed to find target
                # TypeErrors only occur when suppress_exception==True and the function failed to find the color
                # so pass the bad news on, if suppress_exception==False the LostTargetException will continue up
                return None

    def grab_ice(self, frame, suppress_exception = False):
        """
        Detects if the relevant ice is in the gripers, and closes them if it is.
        Must wait until an object enters the grippers
        :param frame: the current frame of the camera
        :return: True if ice was acquired, False otherwise
        """
        if self.debug and not self.is_debug_ignore_state():
            tmp = input("T/F:").__contains__("T")
            if tmp:
                # function success
                return True
            else:
                # function failure
                return False
        else:
            self.navigation_obj.arm_reach()
            self.navigation_obj.straighten_shoulder()
            if self.goal == 0:
                color = self.green_standard
            else:
                color = self.pink_standard
            roi = frame[90:180, 250:320]
            cv.rectangle(frame,(250, 90),(320,180), (255, 0, 0), 3)
            try:
                found = self.find_any_color(roi, suppress_exception)
                if found == color:
                    self.navigation_obj.arm_grab_ice()
                    self.navigation_obj.arm_lower()
                    return True
                else:
                    # ask for correct ice
                    return False
            except LostTargetException or TypeError:
                return False

    def drop_ice(self):
        """
        Continues process to drop the ice in the relevant goal.
        :return: True if ice was dropped, False otherwise
        """
        if self.debug and not self.is_debug_ignore_state():
            tmp = input("T/F:").__contains__("T")
            if tmp:
                # function success
                return True
            else:
                # function failure
                return False
        else:
            # Moves forward to get right up on the box
            self.navigation_obj.move_forward()
            time.sleep(1)
            self.navigation_obj.zero_wheels()
            # drops ice
            self.navigation_obj.arm_raise()
            time.sleep(.5)
            self.navigation_obj.arm_reach()
            time.sleep(.5)
            self.navigation_obj.arm_lower()
            # ice is dropped
            return True

    def find_any_color(self, frame, suppress_exception=False):
        """
        searches the given frame for the standard green and pink, and returns the color it found.
        :param frame: the current frame to search
        :param suppress_exception: if True, the exception will not be raised and the function will return None instead
        :return: The BGR standard for the color detected, or raises a LostTargetException if a color was not found
        """
        if self.debug and not self.is_debug_ignore_state():
            tmp = input("green/pink/lost:")
            if tmp.__contains__("green"):
                # green ice
                return self.green_standard
            elif tmp.__contains__("pink"):
                # pink ice
                return self.pink_standard
            else:
                # lost target
                raise LostTargetException("TESTING, no ice detected")
        else:
            try:
                # look for green
                w, h, loc = self.find_color_in_frame(frame, self.green_standard, suppress_exception)
                return self.green_standard
            except LostTargetException or TypeError:
                # green not found
                try:
                    # look for pink
                    w, h, loc = self.find_color_in_frame(frame, self.pink_standard, suppress_exception)
                    return self.pink_standard
                except LostTargetException or TypeError:
                    # no colors found
                    if suppress_exception:
                        return None
                    else:
                        raise LostTargetException("No ice found")

    def find_color_in_frame(self, frame, color, suppress_exception=False):
        """
        Finds if the color is in the frame and returns the width, height, and center point location
        :param frame: the current camera frame
        :param color: the average color to look for, given as (B, G, R)
        :param suppress_exception: if True, the exception will not be raised and the function will return None instead
        :return: w, h, (x, y); width and height of the detected color area, and the (x, y) location of its center
        """
        value = np.array(color)  # tolerance pivot point
        # threshold on color within the bounds of the tolerance
        detection = cv.inRange(frame, value - self.color_tolerance, value + self.color_tolerance)
        # find the contours of the path area
        contours, _ = cv.findContours(detection, mode=cv.RETR_TREE, method=cv.CHAIN_APPROX_SIMPLE)
        if len(contours) > 0:
            hull = []
            area_hull = []
            # get info on each detected contour
            for i in range(len(contours)):
                hull.append(cv.convexHull(contours[i], False))
                area_hull.append(cv.contourArea(contours[i]))

            # convert to numpy array
            hull = np.array(hull)
            # find the hull with the largest area
            area_max_index = np.argmax(area_hull)
            if area_hull[int(area_max_index)] > 15:
                # find the location of the center of mass
                M = cv.moments(contours[area_max_index])
                try:
                    x = int(M["m10"] / M["m00"])
                    y = int(M["m01"] / M["m00"])
                    # trim hull down to just the hull of interest
                    hull = hull[area_max_index]
                    # find the min and max x values
                    max_x = np.max(hull[:, :, 0])
                    min_x = np.min(hull[:, :, 0])
                    max_y = np.max(hull[:, :, 1])
                    min_y = np.min(hull[:, :, 1])
                    if self.debug:
                        print("color detected, width =", max_x - min_x)
                        cv.circle(frame, (x, y), (max_x-min_x)//2, color=(255, 0, 0), thickness=2)
                    # success, return width of hull and center of mass
                    return max_x - min_x, max_y - min_y, [x, y]
                except ZeroDivisionError:
                    # color not found
                    if suppress_exception:
                        return None
                    else:
                        raise LostTargetException("Color not found in frame")
            else:
                # color not found, area not large enough
                if suppress_exception:
                    return None
                else:
                    raise LostTargetException("Color not found in frame")
        else:
            # color not found
            if suppress_exception:
                return None
            else:
                raise LostTargetException("Color not found in frame")

