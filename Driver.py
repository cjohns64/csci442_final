import cv2 as cv
from StateControl import StateController


class Driver:
    # TODO implement state list
    # find mining area = find_state + target_mining_area

    # travel to mining area = obstacle_avoidance_state + target_mining_area

    # find human = find_state + target_human

    # travel to human = traveling_state + target_human -> ask for ice once there

    # identify ice -> grab ice once correct

    # find start area = find_state + target_start_area

    # return to start = obstacle_avoidance_state + target_start_area

    # find goal area = find_state + target_goal_area for current ice target

    # travel to goal area = traveling_state + target_goal_area -> drop ice once there

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
            obj.main_loop_step(frame)

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
