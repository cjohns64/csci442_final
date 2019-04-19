import numpy as np
import cv2 as cv


class PathDetection:

    def __init__(self, display=False):
        self.display = display

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
        cv.Canny(blur, 125, 255, blur)
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
            width_points = np.argpartition(hull[area_max_index][:,:,1].ravel(), 2)
            width_points = [hull[area_max_index][width_points[0]], hull[area_max_index][width_points[1]]]
            if self.display:
                # draw the two target points
                cv.drawContours(frame, np.array(width_points), -1, (0, 0, 255), thickness=10)
            # remove ugly cv formatting
            width_points = [list(width_points[0][0,:]), list(width_points[1][0,:])]
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

    def get_needed_action(self, target_x_from_center, min_action_value=40):
        """
        Gets whether the robot should rotate or not and in what direction
        :param target_x_from_center: the target horizontal location as measured from the center of the screen
        :param min_action_value: the minimum number of pixels the target must be away from the center for a rotation
        to be needed
        :return: the movement function needed
        """
        if np.abs(target_x_from_center) < min_action_value:
            return self.move_forward  # no rotation
        elif target_x_from_center > 0:
            # rotate left
            return self.rotate_left  # rotate left
        else:
            # rotate right
            return self.rotate_right  # rotate right

    def move_forward(self):
        # TODO add motor movement
        pass

    def rotate_right(self):
        # TODO add motor movement
        pass

    def rotate_left(self):
        # TODO add motor movement
        pass

    def zero_motors(self):
        # TODO add motor movement
        pass
