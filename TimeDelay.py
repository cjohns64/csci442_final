import time


class TimeDelay:

    def __init__(self, delay):
        self.delay = delay
        self.last_time = -1

    def update_time(self):
        """
        Updates the last used time to the current time
        :return:
        """
        self.last_time = time.process_time()

    def check_time(self):
        """
        Checks if it has been less then the delay time since the last update
        :return: True/False
        """
        return not time.process_time() - self.last_time > self.delay
