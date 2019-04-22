import cv2 as cv
from StateControl import StateController
from client import ClientSocket
from picamera.array import PiRGBArray
from picamera import PiCamera
import time

IP = '10.200.22.237'
PORT = 5010
face = None
# set up client and face searching
client = ClientSocket(IP, PORT)

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

    @ staticmethod
    def pi_cam_loop(obj):
        # initialize the camera and grab a reference to the raw camera capture
        camera = PiCamera()
        cv.namedWindow("Video")
        w, h = 320, 240
        camera.resolution = (w, h)  # (640, 480)
        camera.framerate = 32
        rawCapture = PiRGBArray(camera, size=camera.resolution)
        # capture frames from the camera
        for image in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
            # grab the raw NumPy array representing the image, then initialize the timestamp
            # and occupied/unoccupied text
            frame = image.array

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


# start robot state object
robot = StateController(debug=True)
# run w/ laptop camera
Driver.pi_cam_loop(robot)
