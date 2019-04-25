import cv2 as cv
from StateControl import StateController
import numpy as np
from Navigation import Navigation

# import differently for laptop camera
from global_settings import *
if not laptop:
    from picamera.array import PiRGBArray
    from picamera import PiCamera


class Driver:

    @ staticmethod
    def laptop_cam_loop(obj):
        # set up video capture
        cap = cv.VideoCapture(0)
        cv.namedWindow("Video")

        # set screen size
        cap.set(cv.CAP_PROP_FRAME_WIDTH, 400)
        cap.set(cv.CAP_PROP_FRAME_HEIGHT, 300)
        _, frame = cap.read()
        diff32 = np.zeros(frame.shape, np.float32)
        while True:
            # get video info
            _, frame = cap.read()
            frame = cv.GaussianBlur(frame, (9, 9), cv.BORDER_DEFAULT)
            # stabilize image
            cv.accumulateWeighted(frame, diff32, 0.32)
            cv.convertScaleAbs(diff32, frame)

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

    @ staticmethod
    def pi_cam_loop(obj):
        # initialize the camera and grab a reference to the raw camera capture
        camera = PiCamera()
        cv.namedWindow("Video")
        w, h = 320, 240
        camera.resolution = (w, h)  # (640, 480)
        camera.framerate = 32
        rawCapture = PiRGBArray(camera, size=camera.resolution)
        doOnce = True

        # capture frames from the camera
        for image in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
            # grab the raw NumPy array representing the image, then initialize the timestamp
            # and occupied/unoccupied text
            frame = image.array
            if doOnce:
                diff32 = np.zeros(frame.shape, np.float32)
                doOnce = False
            frame = cv.GaussianBlur(frame, (9, 9), cv.BORDER_DEFAULT)
            # stabilize image
            cv.accumulateWeighted(frame, diff32, 0.32)
            cv.convertScaleAbs(diff32, frame)

            # run one frame of the main operating loop
            if obj.main_loop_step(frame):
                # cycle complete
                break

            cv.imshow("Video", frame)

            key = cv.waitKey(1) & 0xFF
            # clear the stream in preparation for the next frame
            rawCapture.truncate(0)

            # if the `q` key was pressed, break from the loop
            if key == ord("q"):
                obj.zero_motors()
                break

    @staticmethod
    def calibrate_color_size(obj):
        width = 0
        height = 0
        n = 0
        if not laptop:
            # initialize the camera and grab a reference to the raw camera capture
            camera = PiCamera()
            cv.namedWindow("Video")
            w, h = 320, 240
            camera.resolution = (w, h)  # (640, 480)
            camera.framerate = 32
            rawCapture = PiRGBArray(camera, size=camera.resolution)
            # capture frames from the camera
            # reach out arm
            nav_obj = Navigation()
            nav_obj.arm_reach()
            for image in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
                # grab the raw NumPy array representing the image, then initialize the timestamp
                # and occupied/unoccupied text
                frame = image.array
                frame = cv.GaussianBlur(frame, (9, 9), cv.BORDER_DEFAULT)
                #color = frame[w//2, h//2]
                color = obj.green_standard
                #color = obj.mining_indicator_standard
                frame = cv.circle(frame, (w//2, h//2), 3, (0,0,255), 5)
                cv.circle(frame, ((w//1.28), (h//1.71)), 30, (0, 255, 255), 2)
                try:
                    wi, hi, loc = obj.find_color_in_frame(frame, color)
                    print(color)
                    width += wi
                    height += hi
                    n += 1
                except:
                    pass

                cv.imshow("Video", frame)

                key = cv.waitKey(1) & 0xFF
                # clear the stream in preparation for the next frame
                rawCapture.truncate(0)

                # if the `q` key was pressed, break from the loop
                if key == ord("q"):
                    obj.zero_motors()
                    break
        print(width//n, height//n)


# start robot state object
robot = StateController(debug=True)
# run w/ laptop/pi camera
if not laptop:
    #Driver.pi_cam_loop(robot)
    robot.navigation_obj.tilt_head_to_move()
    Driver.calibrate_color_size(robot)
    robot.exit()
else:
    Driver.laptop_cam_loop(robot)
    pass
