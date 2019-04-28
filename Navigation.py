import numpy as np
import cv2 as cv
import time
from global_settings import *
if not laptop: import maestro


class Navigation:

    def __init__(self, display=False, debug=False):
        # motor control
        if not laptop: self.tango = maestro.Controller()
        self.BODY = 0
        self.MOTORS = 1
        self.TURN = 2
        self.HEADTURN = 3
        self.HEADTILT = 4
        self.ELBOW = 8
        self.SHOULDER = 6
        self.SHOULDER_SIDE = 7
        self.HAND = 11
        self.hand = 6000
        self.elbow = 5000
        self.shoulder = 6000
        self.shoulder_side = 5600#TODO 5400
        self.body = 6000
        self.headTurn = 6000
        self.headTilt = 6000
        self.motors = 6000
        self.turn = 6000

        # enable/disable displaying the detected path
        self.display = display
        # enable/disable debug printouts/testing
        self.debug = debug
        self.moving_forward = False

        # motor values
        self.motor_step = 200
        self.slow_upper_value = 7000
        self.slow_lower_value = 5000
        self.fast_upper_value = 7000
        self.fast_lower_value = 4600

    def set_arm_motors(self, elbow, hand, shoulder):
        self.ELBOW = elbow
        self.HAND = hand
        self.SHOULDER = shoulder

    def get_zone_lines(self, frame, line_tolerance=20):
        """
        Finds and returns the zone lines that are in the current frame
        :param frame: the current camera frame
        :param line_tolerance: the min distance (in pixels) the line endpoints of each line
        can be from each other before being counted as one line
        :return: the number of detected lines, and a list of zone lines (up to 2), given as an array of [y1, y2]
        and assuming the lines span the frame, or None if no lines where found
        """
        hsv = frame
        saturation = hsv[:, :, 1]  # picks up line really well
        detection = cv.threshold(saturation, 100, 255, cv.THRESH_BINARY)[1]
        # remove noise
        detection = cv.erode(detection, np.ones((5, 5)))
        if self.display: cv.imshow("Detection", detection)
        # get the width of the frame
        w = frame.shape[1]
        # find lines in the image, limiting the min line length should get rid of noise
        lines = cv.HoughLinesP(detection, rho=1, theta=np.pi/180, threshold=175, minLineLength=w*0.75)
        try:
            # show each detected line
            if self.display:
                for line in lines:
                    cv.line(frame, tuple(line[0][:2]), tuple(line[0][2:]), (255, 0, 255))
            # order by y coordinates into 2 sets
            if len(lines[0]) > 1:
                y_ind = np.argpartition(lines[:, 0, 1], 2)
                # take the y coordinates of the start of partition 1 and the end of partition 2
                # and form 2 lines that span the frame
                zone_lines = np.array([(0, lines[y_ind[0]][0][1]), (w, lines[y_ind[0]][0][3]),
                                       (0, lines[y_ind[-1]][0][1]), (w, lines[y_ind[-1]][0][3])])
            else:
                # only one line was detected
                zone_lines = np.array([(0, lines[0][0][1]), (w, lines[0][0][3])])

            # check if 2 lines are truly present,
            print(zone_lines)
            # if there are 2 lines, they will have different y locations
            if len(zone_lines) > 2 and (np.abs(zone_lines[0][0][1] - zone_lines[2][0][1]) < line_tolerance
                                        or np.abs(zone_lines[1][0][1] - zone_lines[3][0][1]) < line_tolerance):
                # one line case
                avg_p1 = int(np.average([zone_lines[0][1], zone_lines[2][1]]))
                avg_p2 = int(np.average([zone_lines[1][1], zone_lines[3][1]]))
                zone_lines = np.array([[0, avg_p1], [w, avg_p2]])
                # display selected line
                if self.display:
                    cv.line(frame, tuple(zone_lines[0]), tuple(zone_lines[1]), (255, 0, 0), thickness=2)

                return 1, zone_lines

            elif len(zone_lines) > 2:  # 2 line case
                # display selected lines
                if self.display:
                    cv.line(frame, tuple(zone_lines[0]), tuple(zone_lines[1]), (255, 0, 0), thickness=2)
                    cv.line(frame, tuple(zone_lines[2]), tuple(zone_lines[3]), (255, 0, 0), thickness=2)

                # return the 2 lines
                return 2, zone_lines
            else:
                # one line not averaged
                if self.display:
                    cv.line(frame, tuple(zone_lines[0]), tuple(zone_lines[1]), (255, 0, 0), thickness=2)
                # return the line
                return 1, zone_lines

        except TypeError or ValueError:
            # no lines where found
            return 0, None

    def get_path(self, frame):
        """
        Gets the target direction from the given frame
        :param frame: the current camera frame
        :return: target pixel location, the location on the screen for the highest unobstructed path
        """
        h, w = frame.shape[:2]
        # default target to the center bottom of the screen
        target_point = (w//2, h)
        # Gaussian Blur
        blur = cv.GaussianBlur(frame, (3, 3), cv.BORDER_DEFAULT)
        # convert to gray scale
        blur = cv.cvtColor(blur, cv.COLOR_BGR2GRAY)
        # Canny
        cv.Canny(blur, 100, 255, blur)
        # vertical fill
        max_row_inds = h - np.argmax(blur[::-1], axis=0)
        row_inds = np.indices((h, w))[0]
        inds_after_edges = row_inds >= max_row_inds
        filled_from_bottom = np.zeros((h, w), np.uint8)
        filled_from_bottom[inds_after_edges] = 255
        # erode
        filled_from_bottom = cv.erode(filled_from_bottom, np.ones((25, 25)))
        filled_from_bottom = cv.dilate(filled_from_bottom, np.ones((25, 25)))

        # find the contours of the path area
        contours, _ = cv.findContours(filled_from_bottom, mode=cv.RETR_TREE, method=cv.CHAIN_APPROX_SIMPLE)

        if self.display:
            cv.drawContours(frame, contours, -1, (255, 0, 0))
        hull = []
        area_hull = []
        # get convex hulls and their areas for all the contours
        for i in range(len(contours)):
            hull.append(cv.convexHull(contours[i], False))
            area_hull.append(cv.contourArea(contours[i]))
        # convert to numpy array
        hull = np.array(hull)
        if self.display:
            cv.drawContours(frame, hull, -1, (0, 255, 0))

        # get max hull and use that
        if len(hull) > 0:
            area_max_index = np.argmax(area_hull)
            # get the highest two points
            width_points = np.argpartition(hull[area_max_index][:, :, 1].ravel(), 2)
            width_points = [hull[area_max_index][width_points[0]], hull[area_max_index][width_points[1]]]
            if self.display:
                # draw the two target points
                cv.drawContours(frame, np.array(width_points), -1, (0, 0, 255), thickness=10)
            # remove ugly cv formatting
            width_points = [list(width_points[0][0, :]), list(width_points[1][0, :])]
            # get target point
            target_point = ((width_points[0][0] + width_points[1][0]) // 2, width_points[0][1])
            if self.display:
                # display target point
                cv.line(frame, (w // 2, h), target_point, (0, 0, 255))

        if self.display:
            # display path area
            tmp = cv.add(np.zeros(frame.shape, np.uint8), 100*np.ones(frame.shape, np.uint8), mask=filled_from_bottom)
            frame = cv.add(tmp, frame)
            cv.imshow("Video", frame)
        # return target point
        return np.array(target_point)

    def get_needed_action(self, target_x_from_center, min_action_value=60):
        """
        Gets whether the robot should rotate or not and in what direction
        :param target_x_from_center: the target horizontal location as measured from the center of the screen
        :param min_action_value: the minimum number of pixels the target must be away from the center for a rotation
        to be needed
        :return: the movement function needed
        """
        if self.debug: print(target_x_from_center, min_action_value)
        if np.abs(target_x_from_center) < min_action_value:
            return self.move_forward  # no rotation
        elif target_x_from_center < 0:
            # rotate left
            return self.rotate_left  # rotate left
        else:
            # rotate right
            return self.rotate_right  # rotate right

    def move_forward(self):
        if self.debug: print("moving forward")
        if move_enabled:
            motor_max = self.fast_lower_value

            # stop turning
            if not self.moving_forward:
                self.zero_wheels()
            self.moving_forward = True
            # one step increase in motor speed
            self.motors -= self.motor_step
            if self.motors < motor_max:
                self.motors = motor_max
            if not laptop: self.tango.setTarget(self.MOTORS, self.motors)

    def rotate_right(self):
        if self.debug: print("rotating right")
        if move_enabled:
            motor_max = self.slow_lower_value
            # stop going forward
            if self.moving_forward:
                self.zero_wheels()
            self.moving_forward = False
            # one step increase in right turning speed
            self.turn -= self.motor_step
            if self.turn < motor_max:
                self.turn = motor_max
            if not laptop: self.tango.setTarget(self.TURN, self.turn)

    def burst_right(self):
        if self.debug: print("burst right")
        if move_enabled:
            for value in range(self.slow_lower_value, self.fast_lower_value, -self.motor_step):
                self.turn = value
                if not laptop: self.tango.setTarget(self.TURN, self.turn)
                time.sleep(0.4)
            self.zero_wheels()

    def turn_180(self):
        if self.debug: print("turning around")
        for _ in range(2):
            self.burst_right()

    def zero_wheels(self):
        if self.debug: print("stopping movement")
        self.turn = 6000
        self.motors = 6000
        if not laptop:
            self.tango.setTarget(self.TURN, self.turn)
            self.tango.setTarget(self.MOTORS, self.motors)

    def rotate_left(self):
        if self.debug: print("rotating left")
        if move_enabled:
            motor_max = self.slow_upper_value
            # stop going forward
            if self.moving_forward:
                self.zero_wheels()
            self.moving_forward = False
            # one step increase in left turning speed
            self.turn += self.motor_step
            if self.turn > motor_max:
                self.turn = motor_max
            if not laptop: self.tango.setTarget(self.TURN, self.turn)

    def tilt_head_to_search(self):
        # zero all motors
        # TODO self.zero_motors()
        if self.debug: print("head tilted up to search")
        # tilt head to searching position
        self.headTilt = 5000
        if not laptop: self.tango.setTarget(self.HEADTILT, self.headTilt)

    def tilt_head_to_move(self):
        # zero all motors
        # TODO self.zero_motors()
        if self.debug: print("head tilted down for movement")
        # tilt head to movement position
        self.headTilt = 4000
        if not laptop: self.tango.setTarget(self.HEADTILT, self.headTilt)

    def tilt_head_to_human(self):
        # zero all motors
        # TODO self.zero_motors()
        if self.debug: print("head tilted up to find human")
        self.headTilt = 7000
        if not laptop: self.tango.setTarget(self.HEADTILT, self.headTilt)

    def arm_reach(self):
        if self.debug: print("arm reached out with open hand")
        self.shoulder = 8000
        self.elbow = 5000
        self.hand = 5000
        if not laptop:
            self.tango.setTarget(self.HAND, self.hand)
            self.tango.setTarget(self.SHOULDER, self.shoulder)
            self.tango.setTarget(self.ELBOW, self.elbow)

    def straighten_shoulder(self):
        if self.debug: print("shoulder straightened")
        self.shoulder_side = 5600
        if not laptop:
            self.tango.setTarget(self.SHOULDER_SIDE, self.shoulder_side)

    def arm_lower(self):
        if self.debug: print("arm lowered without releasing hand")
        self.shoulder_side = 5600
        self.shoulder = 6000
        if not laptop:
            self.tango.setTarget(self.SHOULDER_SIDE, self.shoulder_side)
            time.sleep(.5)
            self.tango.setTarget(self.SHOULDER, self.shoulder)

    def arm_raise(self):
        if self.debug: print("arm raised without releasing hand")
        self.shoulder = 8000
        self.shoulder_side = 4200
        if not laptop:
            self.tango.setTarget(self.SHOULDER, self.shoulder)
            time.sleep(.75)
            self.tango.setTarget(self.SHOULDER_SIDE, self.shoulder_side)

    def arm_grab_ice(self):
        if self.debug: print("arm grabbed ice")
        self.hand = 7700
        if not laptop: self.tango.setTarget(self.HAND, self.hand)

    def zero_motors(self):
        if self.debug: print("zeroing motors")
        # zero all motors
        self.body = 6000
        self.headTurn = 6000
        self.headTilt = 6000
        self.motors = 6000
        self.turn = 6000
        self.arm_lower()
        if not laptop:
            self.tango.setTarget(self.TURN, self.turn)
            self.tango.setTarget(self.MOTORS, self.motors)
            self.tango.setTarget(self.HEADTILT, self.headTilt)
            self.tango.setTarget(self.HEADTURN, self.headTurn)
            self.tango.setTarget(self.BODY, self.body)
