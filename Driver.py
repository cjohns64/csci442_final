import cv2 as cv
from StateControl import StateController


class Driver:

    @ staticmethod
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
            if obj.main_loop_step(frame):
                # cycle complete
                break

            cv.imshow("Video", frame)

            k = cv.waitKey(1)
            # this is the "esc" key
            if k == 27:
                break
        cv.destroyAllWindows()


# start robot state object
robot = StateController(debug=True)
# run w/ laptop camera
Driver.laptop_cam_loop(robot)
