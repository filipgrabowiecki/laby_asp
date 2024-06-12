from threading import Thread, Event
import time
from djitellopy import Tello
import keyboard
import numpy as np
import cv2


class TelloDrone:
    class TelloKillSwitch(Thread):
        tc_handler = None

        def __init__(self, tc_handler):
            Thread.__init__(self)
            self.tc_handler = tc_handler

        def run(self):
            keyboard.wait('space')
            self.tc_handler.force_emergency_stop()

    class Threading(Thread):
        interval = 1.0
        running = None
        func = None

        def __init__(self, interval, event, func):
            Thread.__init__(self)
            self.running = event
            self.interval = interval
            self.func = func

        def run(self):
            while not self.running.wait(self.interval):
                self.func()

    drone = None
    stop_controller = None

    def force_emergency_stop(self):
        self.drone.emergency()
        self.stop_controller.set()

    def camera(self):
        img = self.DRONE_CAMERA.frame

        img = cv2.flip(img, 0)
        img = img[350:, :]
        # img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        lower_blue = np.array([90, 110, 110])
        upper_blue = np.array([130, 255, 255])

        mask = cv2.inRange(hsv, lower_blue, upper_blue)

        edges = cv2.Canny(mask, 50, 150, apertureSize=3)

        cdst = cv2.cvtColor(edges, cv2.COLOR_GRAY2RGB)

        linesP = cv2.HoughLinesP(edges, 1, np.pi / 360, 100, None, 50, 15)
        points = []
        if linesP is not None:
            for i in range(0, len(linesP)):
                l = linesP[i][0]
                points.append(l)
                cv2.line(cdst, (l[0], l[1]), (l[2], l[3]), (0, 0, 255), 3, cv2.LINE_AA)

        if len(points) > 0:
            self.time_lost_line = None

            sorted_points = points
            sorted_points.sort(key=lambda point: point[1])
            lowest_y = sorted_points[0]
            cv2.line(cdst, (lowest_y[0], lowest_y[1]), (lowest_y[2], lowest_y[3]), (255, 0, 0), 3, cv2.LINE_AA)

            first_line = lowest_y
            y_line = first_line[3] - first_line[1]
            if y_line == 0:
                y_line = 0.01
            x_line = first_line[2] - first_line[0]
            tg_angle = x_line / y_line
            angle = np.arctan(tg_angle)
            angle_degree = np.degrees(angle)
            cen_x = lowest_y[0]


            # Rotate only on lines below half of screen
            if -5 > angle_degree:
                print("rotate_right")
                self.order = "rotate_right"

            elif 5 < angle_degree:
                print("rotate_left")
                self.order = "rotate_left"
            else:
                # print("stop_rotate")
                if cen_x <= 410:
                    print("go left")
                    self.order = "go_left"
                elif cen_x >= 550:
                    print("go right")
                    self.order = "go_right"
                else:
                    print("go_straight")
                    self.order = "go_straight"

            # print(abs(angle_degree))
            if abs(angle_degree) > 70:
                # if lowest_y[3] > (720 - 350) / 2:

                if (lowest_y[0] + lowest_y[2]) / 2 < 480:
                    print("ccw")
                    self.order = "ccw"

                else:
                    print("cw")
                    self.order = "cw"


        else:
            # print(f"no lines")
            self.order = "hover"
            current = time.time()
            if self.time_lost_line is None:
                self.time_lost_line = current
            if current >= self.time_lost_line + 3:
                print("landing")
                self.order = "land"
                self.should_land = True

        cv2.imshow('img', cdst)
        cv2.waitKey(1)

    def drone_control(self):
        if self.order is not None:
            if self.order == "land":
                self.drone.send_rc_control(0, 0, 0, 0)
                self.drone.land()
                return

            if self.should_land:
                return

            if self.order == "hover":
                self.drone.send_rc_control(0, 0, 0, 0)

            if self.order == "rotate_right":
                self.drone.send_rc_control(0, 0, 0, 15)
            if self.order == "rotate_left":
                self.drone.send_rc_control(0, 0, 0, -15)
            if self.order == "go_right":
                self.drone.send_rc_control(15, 0, 0, 0)
            if self.order == "go_left":
                self.drone.send_rc_control(-15, 0, 0, 0)
            if self.order == "go_straight":
                self.drone.send_rc_control(0, 15, 0, 0)

            if self.order == "ccw":
                time.sleep(1)
                self.drone.rotate_counter_clockwise(90)
                for i in range(1, 41):
                    self.drone.send_rc_control(0, 15, 0, 0)
                    time.sleep(0.1)

            if self.order == "cw":
                time.sleep(1)
                self.drone.rotate_clockwise(90)
                for i in range(1, 41):
                    self.drone.send_rc_control(0, 15, 0, 0)
                    time.sleep(0.1)

    def main(self):
        self.drone.takeoff()

        _ = self.DRONE_CAMERA.frame
        time.sleep(3)

        camera_thread = self.Threading(0.01, self.stop_controller, self.camera)
        camera_thread.start()

        control_thread = self.Threading(0.1, self.stop_controller, self.drone_control)
        control_thread.start()

    def __init__(self):
        self.drone = Tello()
        self.drone.connect()
        print(f"{self.drone.get_battery()}")

        self.kill_switch = self.TelloKillSwitch(self)
        self.kill_switch.start()
        self.stop_controller = Event()

        self.drone.streamon()
        self.DRONE_CAMERA = self.drone.get_frame_read()

        self.order = None
        self.should_land = False
        self.time_lost_line = None

        self.main()
        time.sleep(300)
        self.drone.land()
        self.drone.end()


if __name__ == "__main__":
    td = TelloDrone()
