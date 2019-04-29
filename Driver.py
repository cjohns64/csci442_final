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
        cv.namedWindow("Detection")

        # set screen size
        cap.set(cv.CAP_PROP_FRAME_WIDTH, 400)
        cap.set(cv.CAP_PROP_FRAME_HEIGHT, 300)

        while True:
            # get video info
            _, frame = cap.read()
            if (obj.get_state_index() != 2) and (obj.get_state_index() != 3):
                frame = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
            # frame = cv.imread("test_images/line_test_image.jpg", cv.IMREAD_COLOR)

            # # run one frame of the main operating loop
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
            if (obj.get_state_index() != 2) and (obj.get_state_index() != 3):
                frame = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
            h, w = frame.shape[:2]

            if doOnce:
                diff32 = np.zeros(frame.shape, np.float32)
                doOnce = False
            if obj.get_frame_blur():
                frame = cv.GaussianBlur(frame, (3, 3), cv.BORDER_DEFAULT)
                # stabilize image
                cv.accumulateWeighted(frame, diff32, 0.80)
                cv.convertScaleAbs(diff32, frame)
                frame = cv.normalize(frame, None, alpha=0, beta=255, norm_type=cv.NORM_MINMAX, dtype=cv.CV_8U)

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
    def calibrate_color_size(obj, sampling=False):
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
                frame = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
                h, w = frame.shape[:2]
                frame = cv.GaussianBlur(frame, (3, 3), cv.BORDER_DEFAULT)
                if sampling:
                    color = frame[h//2, w//2]
                    try:
                        wi, hi, loc = obj.find_color_in_frame(frame, color)
                        cv.circle(frame, (w // 2, h // 2), 15, (0, 0, 255), 5)
                    except:
                        pass
                else:
                    try:
                        wi, hi, loc = obj.find_color_in_frame(frame, obj.pink_standard)
                        print("detected PINK")
                        cv.circle(frame, loc, wi // 2, (255, 0, 0), 2)
                    except:
                        pass
                    try:
                        wi, hi, loc = obj.find_color_in_frame(frame, obj.green_standard)
                        print("detected GREEN")
                        cv.circle(frame, loc, wi // 2, (255, 0, 0), 2)
                    except:
                        pass
                    try:
                        wi, hi, loc = obj.find_color_in_frame(frame, obj.orange_line_standard)
                        print("detected ORANGE")
                        cv.circle(frame, loc, wi // 2, (255, 0, 0), 2)
                    except:
                        pass

                # def get_bgr(event, x, y, flags, params):
                #     global mouseX, mouseY
                #     if event == cv.EVENT_LBUTTONDOWN:
                #         mouseX, mouseY = x, y
                #         print("HSV value")
                #         print(frame[y, x])

                # cv.setMouseCallback('picture', get_bgr, param=frame)

                print("HSV", frame[h//2, w//2])
                cv.imshow("Video", frame)

                key = cv.waitKey(1) & 0xFF
                # clear the stream in preparation for the next frame
                rawCapture.truncate(0)

                # if the `q` key was pressed, break from the loop
                if key == ord("q"):
                    obj.zero_motors()
                    break


# start robot state object
robot = StateController(debug=debug)
# run w/ laptop/pi camera
if not laptop:
    #Driver.pi_cam_loop(robot)
    # robot.navigation_obj.tilt_head_to_move()
    Driver.calibrate_color_size(robot, False)
    robot.exit()
else:
    Driver.laptop_cam_loop(robot)
    pass
# while True:
#     tmp = input("E,H,S:")
#     robot.navigation_obj.zero_motors()
#     values = tmp.split(",")
#     x = []
#     for value in values:
#         x.append(int(value))
#     robot.navigation_obj.set_arm_motors(x[0], x[1], x[2])
#     robot.navigation_obj.arm_reach()

