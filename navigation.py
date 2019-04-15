import numpy as np
import cv2 as cv


class PathDetection:

    def __init__(self):
        cv.namedWindow("Video")
        cv.namedWindow("Canny")
        pass

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
        filled_from_bottom = cv.erode(filled_from_bottom, (500, 500))
        # Smooth Hull
        # hull = cv.convexHull(filled_from_bottom)
        tmp = cv.add(np.zeros(frame.shape, np.uint8), 100*np.ones(frame.shape, np.uint8), mask=filled_from_bottom)
        frame = cv.add(tmp, frame)
        cv.imshow("Video", frame)
