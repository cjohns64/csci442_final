import cv2 as cv
import navigation as nav


class Driver:

    def __init__(self):
        # set up video capture
        self.cap = cv.VideoCapture(0)
        cv.namedWindow("Video")

        # set screen size
        self.cap.set(cv.CAP_PROP_FRAME_WIDTH, 400)
        self.cap.set(cv.CAP_PROP_FRAME_HEIGHT, 300)

        # set up objects
        self.navigation_obj = nav.PathDetection(display=True)
        self.move_function = None

    def obstacle_avoidance_state(self, frame):
        """
        State for navigating through the obstacles
        :param frame: the current frame of the camera
        :return:
        """
        if frame is not None:
            # get path target location on the screen
            nav_x, nav_y = self.navigation_obj.get_path(frame)
            # get the movement function we need
            self.move_function = self.navigation_obj.get_rotation(nav_x - frame.shape[1] // 2)
            # TODO use the movement function
            pass

    def main_loop(self):
        while True:
            # get video info
            status, frame = self.cap.read()

            # TODO transition between states
            self.obstacle_avoidance_state(frame)

            cv.imshow("Video", frame)

            k = cv.waitKey(1)
            # this is the "esc" key
            if k == 27:
                break
        cv.destroyAllWindows()


robot = Driver()
robot.main_loop()