import numpy as np
import cv2 as cv


class PathDetection:

    def __init__(self):
        cv.namedWindow("Video")

    def get_path(self, frame):
        h, w = frame.shape[:2]
        # Gaussian Blur
        blur = cv.GaussianBlur(frame, (3, 3), cv.BORDER_DEFAULT)
        # convert to gray scale
        blur = cv.cvtColor(blur, cv.COLOR_BGR2GRAY)
        # Floor Finder?
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
        # Smooth Hull
        # find direction
        contours, _ = cv.findContours(filled_from_bottom, mode=cv.RETR_TREE, method=cv.CHAIN_APPROX_SIMPLE)
        cv.drawContours(frame, contours, -1, (255, 0, 0))
        hull = []
        area_hull = []
        for i in range(len(contours)):
            hull.append(cv.convexHull(contours[i], False))
            area_hull.append(cv.contourArea(contours[i]))
        hull = np.array(hull)
        cv.drawContours(frame, hull, -1, (0, 255, 0))
        # get max hull and use that
        area_max_index = np.argmax(area_hull)
        width_points = np.argpartition(-hull[area_max_index][:,:,0].ravel(), 2)
        cv.drawContours(frame, hull[area_max_index], -1, (0, 0, 255), thickness=2)

        tmp = cv.add(np.zeros(frame.shape, np.uint8), 100*np.ones(frame.shape, np.uint8), mask=filled_from_bottom)
        frame = cv.add(tmp, frame)
        cv.imshow("Video", frame)
