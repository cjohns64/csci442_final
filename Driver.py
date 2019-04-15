import cv2 as cv
import numpy as np

cap = cv.VideoCapture(0)
cv.namedWindow("Video")
cv.namedWindow("image1")
cv.namedWindow("absDiff")

cap.set(cv.CAP_PROP_FRAME_WIDTH, 400)
cap.set(cv.CAP_PROP_FRAME_HEIGHT, 300)
# get video info
status, frame = cap.read()
width, height, channel = frame.shape

# diff32 = np.zeros((width, height, 3), np.float32)
# mask = 20*np.ones((width, height, 3), np.uint8)


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


while True:
    # get video info
    status, frame = cap.read()

    if frame is not None:
        # update image size
        width, height, channel = frame.shape
        # if mask.shape[:2] != frame.shape[:2]:
        #     mask = 20 * np.ones((width, height, 3), np.uint8)
        # if diff32.shape[:2] != frame.shape[:2]:
        #     diff32 = np.zeros((width, height, 3), np.float32)

        # # brighten image
        # tmp_frame = cv.add(frame, mask)
        # # blur image
        # blur = cv.GaussianBlur(tmp_frame, (3, 3), cv.BORDER_DEFAULT)
        # # accumulate frames
        # cv.accumulateWeighted(blur, diff32, 0.32)
        # cv.convertScaleAbs(diff32, blur)
        # # take difference
        # diff = cv.absdiff(blur, frame)
        # # convert to grayscale
        # diff = cv.cvtColor(diff, cv.COLOR_BGR2GRAY)
        #
        # # threshold image
        # _, thresh1 = cv.threshold(diff, 30, 255, cv.THRESH_BINARY)
        # thresh2 = cv.GaussianBlur(diff, (9, 9), cv.BORDER_DEFAULT)
        # _, thresh2 = cv.threshold(thresh2, 30, 255, cv.THRESH_BINARY)
        #
        # # find contours
        # contours, _ = cv.findContours(thresh2, cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE)
        # if contours is not None:
        #     for shape in contours:
        #         # draw a blob and a bounding box for every significant blob
        #         if len(shape) > 300:
        #             draw_blob(diff, shape)
        #             draw_blob(frame, bounding_box(shape))

        # display results
        cv.imshow("Video", frame)
        # cv.imshow("image1", thresh2)
        # cv.imshow("absDiff", diff)

    k = cv.waitKey(1)
    # this is the "esc" key
    if k == 27:
        break
cv.destroyAllWindows()