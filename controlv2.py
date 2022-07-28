## VISUAL SERVOING CONTROLLER ##

from thorlabsBSC203 import BSC203
from yolo5v2 import Vision, Modality
from pynput import keyboard
import time
import numpy as np
from threading import Thread


class Control:

    def __init__(self, tolerance=0.001):
        self.visionSystem = Vision()
        self.controller = BSC203(update_time=10)
        self.center_tolerance = tolerance
        self.modality = Modality.Observation
        self.stop = False
        self.first_catch = False




    def initialization(self, start):
        self.controller.resume_move_messages(1)
        self.controller.resume_move_messages(2)
        self.controller.resume_move_messages(3)

        self.controller.homing_all()
        print('homed')

        self.controller.controller.clear_message_queue(1)
        self.controller.controller.clear_message_queue(2)

        self.controller.goto_absolute_position_group(start)

        self.set_modality(Modality.Observation)


    def registration(self, movement):
        x = -movement[0]
        y = movement[1]
        return [x, y]

    def unity_transformation(self, movement):
        x = movement[0] * self.visionSystem.pixel_width * self.visionSystem.resolution / 10000
        y = movement[1] * self.visionSystem.pixel_height * self.visionSystem.resolution / 10000
        x = round(x, 3)
        y = round(y, 3)
        return [x, y]

    def set_modality(self, mode):
        self.visionSystem.mode.value = mode.value
        self.modality = mode

        if mode == Modality.Observation:
            self.controller.set_jog_mode(1, 'Continuous', 'Profiled')
            self.controller.set_jog_mode(2, 'Continuous', 'Profiled')
            self.controller.set_jog_mode(3, 'Continuous', 'Profiled')

            self.controller.set_jog_vel_params(1, 1, 1)
            self.controller.set_jog_vel_params(2, 1, 1)
            self.controller.set_jog_vel_params(3, 0.05, 1)

            self.visionSystem.found.value = 0

        if mode == Modality.Research:
            self.controller.suspend_move_messages(1)
            self.controller.suspend_move_messages(2)

            self.controller.set_vel_params(1, 1, 3)
            self.controller.set_vel_params(2, 1, 3)

            print('RESEARCHING')

        if mode == Modality.Following:
            print('FOLLOWING')
            self.controller.suspend_move_messages(1)
            self.controller.suspend_move_messages(2)

            self.controller.set_jog_vel_params(1, 1, 1)
            self.controller.set_jog_vel_params(2, 1, 1)

            # self.controller.set_vel_params(1, 10, 40)
            # self.controller.set_vel_params(2, 10, 40)

    def on_press(self, key):
        if self.modality == Modality.Observation:
            try:
                if key.char == 'w':
                    self.controller.jogging(2, 'Forwards')

                elif key.char == 's':
                    self.controller.jogging(2, 'Reverse')

                elif key.char == 'a':
                    self.controller.jogging(1, 'Reverse')

                elif key.char == 'd':
                    self.controller.jogging(1, 'Forwards')

                elif key.char == 'z':
                    self.controller.jogging(3, 'Forwards')

                elif key.char == 'x':
                    self.controller.jogging(3, 'Reverse')

                elif key.char == 'r':
                    self.research_starting_point = self.controller.get_position_group()

                    self.points_research = self.research_routine()
                    self.points_research = [[point[0] + self.research_starting_point[0], point[1] + self.research_starting_point[1], point[2]] for point in self.points_research]

                    self.set_modality(Modality.Research)

            except:
                print('Use wasd to move the stage!')

        if key.char == 'm':
            self.set_modality(Modality.Observation)

        if key == keyboard.Key.esc:
            print('CLOSING...')
            self.stop = True



    def on_release(self, key):
        if self.modality == Modality.Observation:
            try:
                if key.char == 'w' or key.char == 's':
                    self.controller.stop_profiled(2)

                elif key.char == 'a' or key.char == 'd':
                    self.controller.stop_profiled(1)

                elif key.char == 'x' or key.char == 'z':
                    self.controller.stop_profiled(3)

            except:
                print('Unable to stop now')

        if key == keyboard.Key.esc:
            self.stop = True


    def research_routine(self):
        n = 500
        points = [np.array([]) for _ in range(n)]
        angle = np.linspace(0,  14 * 2 * np.pi, n)
        radius = np.linspace(0, 2, n)

        x = radius * np.cos(angle)
        y = radius * np.sin(angle)

        for i in range(n):
            points[i] = [x[i], y[i], self.research_starting_point[2]]

        return points

    def follow(self):
        # get the error from the camera
        error = self.visionSystem.position_error
        # transform the error in S.I. units
        error_si = self.unity_transformation(error)
        # change the reference system from camera to stage
        error_stage = self.registration(error_si)

        if not self.first_catch:
            # send command to the controller's axis
            for axes in range(2):
                if abs(error_stage[axes]) > (self.center_tolerance + 1*0.001 + 0.5*0.001**2):
                    if error_stage[axes] > 0:
                        self.controller.jogging(axes+1, 'Reverse')
                    else:
                        self.controller.jogging(axes+1, 'Forwards')
                else:
                    self.controller.stop_profiled(axes+1)
        else:
            error_stage.append(0.0)
            self.controller.relative_movement_group(error_stage)
            self.first_catch = False



    def main(self):
        # self.initialization([15, 33, 11.4])


        self.listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release)

        self.listener.start()

        # control_signal = Thread(target=sle)


        while not self.stop:

            if self.modality == Modality.Following:
                self.follow()
                time.sleep(0.020)

            elif self.modality == Modality.Research:
                # until a sample is found
                while not self.visionSystem.found.value:
                    # perform a spiral
                    for point in self.points_research:
                        self.controller.goto_absolute_position_group(point)
                        time.sleep(0.001)
                        print('SPIRAL')
                        if self.visionSystem.found.value:
                            print('found!!!!!!!!')
                            self.controller.set_vel_params(1, 4, 20)
                            self.controller.set_vel_params(2, 4, 20)
                            self.follow()
                            break

                    # # after the spiral is finished came back to the initial position
                    # self.controller.goto_absolute_position_group(self.research_starting_point)

                self.set_modality(Modality.Following)

            else:
                time.sleep(0.001)

            time.sleep(0.001)
            # print(str(self.modality))

        # self.listener.join()
        self.controller.stop_all()
        print('STOPPED')
        self.listener.stop()





if __name__ == '__main__':
     control = Control()

     control.visionSystem.start()

     control.main()

     control.visionSystem.join()

     control.controller.stop_all()
     control.controller.stop_polling()


