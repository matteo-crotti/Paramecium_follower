"""
This class communicates with a Thorlabs
Benchtop Stepper Motor Controller (BSC203).
"""
import os
import time
import threading

from msl.equipment import (
    EquipmentRecord,
    ConnectionRecord,
    Backend,
)

from msl.equipment.resources.thorlabs import MotionControl



# ensure that the Kinesis folder is available on PATH
os.environ['PATH'] += os.pathsep + 'C:/Program Files/Thorlabs/Kinesis'

class BSC203():

    def __init__(self, update_time):
        self.record = EquipmentRecord( #data of our device
            manufacturer='Thorlabs',
            model='BSC203',
            serial='70224524',
            connection=ConnectionRecord(
                address='SDK::Thorlabs.MotionControl.Benchtop.StepperMotor.dll',
                backend=Backend.MSL,
            )
        )

        self.controller = None #connection to the controller
        self.connect()

        self.channelsNo = self.controller.max_channel_count()  # number of channels of the controller

        for channel in range(self.channelsNo):
            self.controller.load_settings(channel+1) #settings of each motor
            # self.controller.start_polling(channel+1, update_time)

        self.poller = [None] * self.channelsNo
        self.communicator = [None] * self.channelsNo
        self.polling(update_time)




    def connect(self):
        MotionControl.build_device_list() #avoid the FT_DeviceNotFound error
        #connect to the Benchtop Stepper Motor
        self.controller = self.record.connect()
        print('Connected to {}'.format(self.controller))
        # the SBC_Open(serialNo) function in Kinesis is non-blocking and therefore we should add a delay for Kinesis to establish communication with the serial port
        time.sleep(1)



    def wait(self, channel, msg_code):
        """
        The controller works with serial port commands. It can not execute a command on a channel if the previous one has not been terminated.
        This function allow to wait until the channel executes the command.

        :param channel: the channel we want to communicate with
        :param msg_code: the error code we are waiting for

        """
        message_type, message_id, _ = self.controller.wait_for_message(channel)
        while message_type != 2 or message_id != msg_code:
            message_type, message_id, _ = self.controller.wait_for_message(channel)


    def polling(self, update_time):
        """
        Generates a separate thread for each channel to continuously request position and status

        """
        for channel in range(self.channelsNo):
            self.poller[channel] = threading.Thread(target=self.controller.start_polling(channel+1, update_time))  # create a thread for polling at 100ms
            self.poller[channel].start()
            # self.poller[channel].join()

    def stop_polling(self):
        """
        Stops the polling thread related to each channel
        :return:
        """
        for channel in range(self.channelsNo):
            self.controller.stop_polling(channel+1)
            self.poller[channel].join()

    def usb_communication(self, channel, msg_code):
        """
        Generates a thread for the selected channel to read from the usb
        :return:
        """

        self.communicator[channel-1] = threading.Thread(target=self.wait(channel, msg_code))
        self.communicator[channel-1].start()
        self.communicator[channel-1].join()




    def homing(self, channel):
        """
        Homes the stage channel to a known position.

        :param channel: channel to be homed
        :return:
        """

        #First check if the channel can home
        if self.controller.can_home(channel):
            self.controller.home(channel)
            self.usb_communication(channel, 0)


    def homing_all(self):
        """
        Homes the all the stage's channel to a known position.

        :param channel: channel to be homed
        :return:
        """
        for channel in range(self.channelsNo):
            if self.controller.can_home(channel+1):
                self.controller.home(channel+1)

        self.controller.clear_message_queue(channel)
        for channel in range(self.channelsNo):
            self.usb_communication(channel+1, 0)



    # THE FUNCTION ARE DESINGED TO WORK WITH THE CONTROLLER UNIT (STEPS) SO TO GIVE THE REAL UNIT WE NEED TO CONVERT IT

    def get_position(self, channel):
        return self.controller.get_real_value_from_device_unit(channel, self.controller.get_position(channel), 0)


    def get_position_group(self):
        """
        Get the position of all the axes

        :return: A vector containing the position of all the axes
        """
        position = [0] * self.channelsNo
        for channel in range(self.channelsNo):
            position[channel] = self.controller.get_real_value_from_device_unit(channel+1, self.controller.get_position(channel+1), 0)

        return position


    def goto_absolute_position(self, channel, position): #position in nm
        """
        Moves the axes of the specified channel to the desired absolute position.

        :param channel: the channel to be moved
        :param position: the absolute position we want the stage to go
        :return:
        """
        devUnit_position = self.controller.get_device_unit_from_real_value(channel, position, 0)
        self.controller.move_to_position(channel, devUnit_position)

        self.controller.clear_message_queue(channel)
        self.usb_communication(channel, 1)



    def goto_absolute_position_group(self, position):
        """
        Moves the stage to the desired absolute position.

        :param position: the absolute position we want the stage to go
        :return:
        """
        for channel in range(self.channelsNo):
            devUnit_position = self.controller.get_device_unit_from_real_value(channel+1, position[channel], 0)
            self.controller.move_to_position(channel+1, devUnit_position)

        self.controller.clear_message_queue(channel)
        for channel in range(self.channelsNo):
            self.usb_communication(channel+1, 1)



    def relative_movement(self, channel, distance):
        """
        Moves the axes of the specified channel by the specified distance from the current position.

        :param channel: the channel to be moved
        :param position: the distance to be moved through
        :return:
        """
        devUnit_distance = self.controller.get_device_unit_from_real_value(channel, distance, 0)
        self.controller.move_relative(channel, devUnit_distance)

        self.controller.clear_message_queue(channel)
        self.usb_communication(channel, 1)


    def relative_movement_group(self, distance):
        """
        Moves the stage by the specified distance from the current position.

        :param distance: the distance to be moved through the different axes
        :return:
        """
        for channel in range(self.channelsNo):
            devUnit_distance = self.controller.get_device_unit_from_real_value(channel + 1, distance[channel], 0)
            self.controller.move_relative(channel + 1, devUnit_distance)

        self.controller.clear_message_queue(channel)
        for channel in range(self.channelsNo):
            self.usb_communication(channel + 1, 1)


    def jogging(self, channel, direction):
        """
        Perform a fixe distance step in the direction of the selected channel.

        :param channel: the channel we want to move.
        :param direction: perform a step forwards or backwards
        :return:
        """
        self.controller.move_jog(channel,direction)

        [mode, stop] = self.controller.get_jog_mode(channel)
        if mode == 'SingleStep':
            self.controller.clear_message_queue(channel)
            self.usb_communication(channel, 1)
        else:
            self.controller.suspend_move_messages(channel)


    def set_jog_step_size(self, channel, step_size):
        """

        :param channel: the channel to set
        :param step_size: the desired step size
        :return:
        """
        devUnit_step_size = self.controller.get_device_unit_from_real_value(channel, step_size, 0)
        self.controller.set_jog_step_size(channel, devUnit_step_size)


    def set_jog_vel_params(self, channel, max_velocity, acceleration):
        """
        Sets the jog velocity parameters.

        :param channel: the channel whose velocity has to be set
        :param max_velocity: maximum velocity in real value
        :param acceleration: acceleration in real value
        :return:
        """
        devUnit_max_velocity = self.controller.get_device_unit_from_real_value(channel, max_velocity, 1)
        devUnit_acceleration = self.controller.get_device_unit_from_real_value(channel, acceleration, 2)

        self.controller.set_jog_vel_params(channel, devUnit_max_velocity, devUnit_acceleration)



    #maybe useful for velocity control
    def move_at_velocity(self, channel, direction):
        """
        Make the platform move at specific velocity in the desired direction

        :param channel: the channel to be moved
        :param direction: forwards or backwards
        :return:
        """
        self.controller.move_at_velocity(channel, direction)


    def set_vel_params(self, channel, max_velocity, acceleration):
        """
        Sets the move velocity parameters.

        :param channel: the channel whose velocity has to be set
        :param max_velocity: maximum velocity in real value
        :param acceleration: acceleration in real value
        :return:
        """
        devUnit_max_velocity = self.controller.get_device_unit_from_real_value(channel, max_velocity, 1)
        devUnit_acceleration = self.controller.get_device_unit_from_real_value(channel, acceleration, 2)

        self.controller.set_vel_params(channel, devUnit_max_velocity, devUnit_acceleration)




    #STOP ACTIONS
    def stop_profiled(self, channel):
        """
        Stop the current move usign the current velocity profile.

        :param channel: the channel to stop
        :return:
        """
        self.controller.stop_profiled(channel)

        # self.controller.clear_message_queue(channel)
        # self.usb_communication(channel, 2)



    def stop_immediate(self, channel):
        """
        Stop the current move immediately (with the risk of loosing position)

        :param channel: the channel to stop
        :return:
        """
        self.controller.stop_immediate(channel)

        # self.controller.clear_message_queue(channel)
        # self.usb_communication(channel, 2)

    def stop_all(self):
        """
        Stop the current move immediately (with the risk of loosing position) for all axes

        :return:
        """
        for channel in range(self.channelsNo):
            self.controller.stop_profiled(channel+1)

        # self.controller.clear_message_queue(channel)
        # for channel in range(self.channelsNo):
        #     self.usb_communication(channel + 1, 2)



    #could be useful
    def set_jog_mode(self, channel, mode, stop_mode):
        """
        Change the jog mode in continuous or single step and select the stop mode.

        :param channel: the channel to be setted
        :param mode: continuous or single step
        :param stop_mode: immediate or profiled
        :return:
        """
        self.controller.set_jog_mode(channel, mode, stop_mode)

    def suspend_move_messages(self, channel):
        """
        Suspend automatic messages at the end of the moves.

        :param channel: the channel number
        :return:
        """
        self.controller.suspend_move_messages(channel)

    def resume_move_messages(self, channel):
        """
        Resume suspended move messages.

        :param channel: the channel number.
        :return:
        """
        self.controller.resume_move_messages(channel)

    # use getters as control



if __name__ == '__main__':
    bsc = BSC203()

    bsc.homing_all()















