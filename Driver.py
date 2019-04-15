import cv2 as cv
import numpy as np
import navigation as nav

cap = cv.VideoCapture(0)
cv.namedWindow("Video")

cap.set(cv.CAP_PROP_FRAME_WIDTH, 400)
cap.set(cv.CAP_PROP_FRAME_HEIGHT, 300)
# get video info
status, frame = cap.read()
width, height, channel = frame.shape


def draw_blob(img, verts):
    """
    Draws a line around a area given by the list of vertexes, given in the weird OpenCV list of [[x, y]]
    :param img: the image to draw the blob on
    :param verts: the bounds of the blob given in the order around the blob
    :return: None
    """
    for i in range(len(verts) - 1):
        v1 = tuple(verts[i][0])
        v2 = tuple(verts[i+1][0])
        cv.line(img, v1, v2, (255, 0, 0), 3)
    cv.line(img, tuple(verts[-1][0]), tuple(verts[0][0]), (255, 0, 0), 3)


def bounding_box(verts):
    """
    Returns the bounding box for a blob given as a list of vertexes as a list of [[x, y]]
    the returned list of vertexes of the bounding box are also in this form.
    :param verts: the bounds of the blob given in the order around the blob
    :return: the bounds of the bounding box
    """
    min_x, min_y = np.min(verts[:,0], axis=0)
    max_x, max_y = np.max(verts[:,0], axis=0)

    return np.array([[[min_x, min_y]], [[max_x, min_y]], [[max_x, max_y]], [[min_x, max_y]]])


x = nav.PathDetection()

while True:
    # get video info
    status, frame = cap.read()

    if frame is not None:
        x.get_path(frame)

    k = cv.waitKey(1)
    # this is the "esc" key
    if k == 27:
        break
cv.destroyAllWindows()