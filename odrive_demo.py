#!/usr/bin/python
"""
Interface from Python to ODrive
Daniel J. Gonzalez - dgonz@mit.edu
2.12 Intro to Robotics Spring 2019

Edited by Rachel Hoffman-Bice and Jerry Ng, January 2020
"""

import odrive
from odrive.enums import *
import time
import math
import fibre
import serial
import struct
import signal
import sys
import pdb

pi = 3.1415927
pi
in2mm = 25.4
mm2in = 1/in2mm
in2m = in2mm/1000

Nm2A = 0.00000604 

class Odrive:
    def __init__(self,usb_serial,axis_num):
        self.usb_serials=usb_serial
        self.axis_num = axis_num
        self.axis1 = axis_num
        self.axis1 = None
        self.offsets = [[[-8.59,-6.11],[-3.61,5.89]]]
        self.zeroVec = [[[0,0],[0,0]]]
        self.thtDesired = [[[0,0],[0,0]]]
        self.velDesired = [[[0,0],[0,0]]]
        self.kP = [[[0,0],[0,0]]]
        self.kD = [[[0,0],[0,0]]]
        self.home_kp = [[[0,0],[0,0]]]
        self.home_kd = [[[0,0],[0,0]]]
        self.kPd = [[[0,0],[0,0]]]
        self.kDd = [[[0,0],[0,0]]]

        self.CPR2RAD = (2*math.pi/400000)
        self.connect_all()
        self.set_gains(kp = 0.1, kd = 0.0001)
        #self.full_init()


    def rad2Count(self,angle):
        try:
            return [100000-ang/self.CPR2RAD for ang in angle]
        except TypeError:
            return 100000-angle/self.CPR2RAD

    def count2Rad(self,count):
        try:
            return [(100000-cnt)*self.CPR2RAD for cnt in count]
        except TypeError:
            return (100000-count)*self.CPR2RAD

    def connect_all(self):
        print("Finding odrive: " + self.usb_serials+ "...")

        odrv = odrive.find_any(serial_number = self.usb_serials)

        print("Found odrive!")

        self.odrv=odrv
        if self.axis_num == 1:
            self.axis1 = odrv.axis1
        elif self.axis_num == 0:
            self.axis1 = odrv.axis1

    def get_cnt(self):
        positions = 0
        positions = self.axis1.encoder.pos_estimate
        return positions

    def get_rad_all(self):
        return count2Rad(get_cnt_all(self))

    def print_controllers(self):
        print(self.axis1.controller)

    def print_encoders(self):
        print(self.axis1.encoder)

    def printErrorStates(self):
        print(' axis error:',hex(self.axis1.error))
        print( ' motor error:',hex(self.axis1.motor.error))
        print( ' encoder error:',hex(self.axis1.encoder.error))


    def printPos(self):

        print(' pos_estimate: ', self.axis1.encoder.pos_estimate)
        print(' count_in_cpr: ',self.axis1.encoder.count_in_cpr)
        print(' shadow_count: ', self.axis1.encoder.shadow_count)


    def print_all(self):
        self.printErrorStates()
        self.print_encoders()
        self.print_controllers()


    def reboot(self):

        self.odrv.reboot()
        time.sleep(5)
        connect_all()
        print('Rebooted ')

    def trajMoveCnt(self,posDesired = 10000, velDesired = 50000, accDesired = 50000):

        self.axis1.trap_traj.config.vel_limit = velDesired 
        self.axis1.trap_traj.config.accel_limit = accDesired 
        self.axis1.trap_traj.config.decel_limit = accDesired
        self.axis1.controller.move_to_pos(posDesired)


    def trajMoveRad(self,posDesired = 0, velDesired = 2*pi/8, accDesired = 2*pi/8):
        trajMoveCnt(self,rad2Count(posDesired), rad2Count(velDesired), rad2Count(accDesired))

    def pos_test_one(self, amt = 400000, mytime = 5):
        print('Changing modes to position control.')
        self.axis1.requested_state = AXIS_STATE_IDLE

        self.axis1.controller.config.control_mode = CTRL_MODE_POSITION_CONTROL
        self.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        print('Moving in... 3')
        time.sleep(1)
        print('2')
        time.sleep(1)
        print('1')
        time.sleep(1)
        self.axis1.controller.pos_setpoint = amt
        time.sleep(mytime)

        self.axis1.controller.pos_setpoint = 0
        time.sleep(mytime)

        self.axis1.controller.pos_setpoint = amt
        time.sleep(mytime)

        self.axis1.controller.pos_setpoint = 0
        time.sleep(mytime)

        self.axis1.requested_state = AXIS_STATE_IDLE


    def vel_test_one(self, amt = 400000, mytime = 5):
        self.axis1.requested_state = AXIS_STATE_IDLE
        print('Changing modes to velocity control.')
        self.axis1.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL

        self.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.axis1.controller.vel_setpoint = 0
        print('Moving in... 3')
        time.sleep(1)
        print('2')
        time.sleep(1)
        print('1')
        time.sleep(1)

        self.axis1.controller.vel_setpoint = amt
        time.sleep(mytime)

        self.axis1.controller.vel_setpoint = 0
        time.sleep(mytime)

        self.axis1.controller.vel_setpoint = -amt
        time.sleep(mytime)

        self.axis1.controller.vel_setpoint = 0
        time.sleep(mytime)

        self.axis1.requested_state = AXIS_STATE_IDLE


    def traj_test_one(self, amt = 400000, vel_limit = 100000, accel_limit = 10000, mytime = 10):
        self.axis1.requested_state = AXIS_STATE_IDLE
        print('Changing modes to trajectory control.')

        self.axis1.trap_traj.config.vel_limit = vel_limit
        self.axis1.trap_traj.config.accel_limit = accel_limit
        self.axis1.trap_traj.config.decel_limit = accel_limit
        self.axis1.controller.config.control_mode = CTRL_MODE_POSITION_CONTROL
        self.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

        print('Moving in... 3')
        time.sleep(1)
        print('2')
        time.sleep(1)
        print('1')
        time.sleep(1)
        self.axis1.controller.move_to_pos(amt)
        time.sleep(mytime)

        self.axis1.controller.move_to_pos(0)
        time.sleep(mytime)

        self.axis1.controller.move_to_pos(amt)
        time.sleep(mytime)

        self.axis1.controller.move_to_pos(0)
        time.sleep(mytime)

        self.axis1.requested_state = AXIS_STATE_IDLE

    def erase_and_reboot(self):
        print('erasing config')
        self.odrv.erase_configuration()
        print('reboot')
        self.odrv.reboot()


    def startup_init(self, verbose = False):
        if(verbose == True):
            print("Startup Init")
            self.axis1.requested_state = AXIS_STATE_IDLE
            time.sleep(5)
            self.printErrorStates()
            print('axis state idle')
            self.axis1.requested_state = AXIS_STATE_ENCODER_INDEX_SEARCH
            time.sleep(10)
            self.printErrorStates()
            print('index search')
            self.axis1.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
            time.sleep(20)
            self.printErrorStates()
            print('offset calibration')
            self.axis1.requested_state = AXIS_STATE_IDLE
            time.sleep(10)
            self.printErrorStates()
            print('axis state idle')
            self.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            time.sleep(5)
            self.printErrorStates()
            print('closed loop control')

            print('Startup Init Completed')
            self.initflag=1
        else:
            self.axis1.requested_state = AXIS_STATE_IDLE
            time.sleep(5)
            self.axis1.requested_state = AXIS_STATE_ENCODER_INDEX_SEARCH
            time.sleep(10)
            self.axis1.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
            time.sleep(20)
            self.axis1.requested_state = AXIS_STATE_IDLE
            time.sleep(10)
            self.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            time.sleep(5)
            self.initflag=1

    def full_init(self,reset = True):
        self.printErrorStates()
        if(reset):
            self.odrv.config.brake_resistance = 0
            self.axis1.motor.config.pre_calibrated = False

            #pole pairs
            self.axis1.motor.config.pole_pairs = 4
            self.axis1.controller.config.vel_limit = 200000 

            self.axis1.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT
            self.axis1.encoder.config.cpr = 4000
            self.axis1.encoder.config.use_index = True
            self.axis1.encoder.config.zero_count_on_find_idx = True
            self.axis1.encoder.config.pre_calibrated = False

            #motor calibration current
            self.axis1.motor.config.calibration_current = 4
            time.sleep(1)
            self.printErrorStates()
            self.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
            print('fullcalib')
            self.printErrorStates()
            time.sleep(10)
            self.printErrorStates()
        time.sleep(10)
        self.printErrorStates()
        self.axis1.requested_state = AXIS_STATE_IDLE
        self.axis1.motor.config.pre_calibrated=True

        self.axis1.config.startup_encoder_index_search = True
        self.axis1.config.startup_encoder_offset_calibration = True
        self.axis1.controller.config.control_mode = CTRL_MODE_POSITION_CONTROL
        self.printErrorStates()
        kP_des = 0.1
        kD_des = 0.0001
        self.axis1.controller.config.pos_gain = kP_des 
        self.axis1.controller.config.vel_gain = kD_des
        self.axis1.controller.config.vel_integrator_gain = 0
        self.axis1.controller.pos_setpoint = 0
        time.sleep(1)

        self.odrv.save_configuration()
        time.sleep(2)
        print('all done')
        self.printErrorStates()


    def set_gains(self,kp,kd,ki = 0,make_permanent = 0):
         self.axis1.requested_state=AXIS_STATE_IDLE
         self.axis1.controller.config.pos_gain = kp
         self.axis1.controller.config.vel_gain = kd
         self.axis1.controller.config.vel_integrator_gain = ki
         time.sleep(1)
         if make_permanent==1:
             self.odrv.save_configuration()

    def make_perm(self):
        self.odrv.save_configuration()

    def set_closed_loop_state(self):
        self.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

    def test_traj_pos(self):
        self.pos_test_one()

    def test_traj_vel(self):
        self.vel_test_one()

    def test_traj(self):
        self.traj_test_one()

if __name__ == '__main__':
    serial_id = str(raw_input('Type in the serial number for your ODrive: \n'))
    #Ex: 2086378C3548
    axis_number = input('Which axis number is your motor connected to? \n')
    #Ex: 1 or 0

    while 1:
        if(axis_number == 1 or axis_number == 0):
            break
        else:
            print('You entered an impossible entry.')
            axis_number = input('Please enter either 0 or 1. \n')
    
    print('Attempting to connect to ODrive with serial number: ' + str(serial_id))
    motor_driver = Odrive(usb_serial = serial_id, axis_num = axis_number)

    print('The motor driver has completed connection and calibration. It is ready to run.')
    raw_input('Press enter to drive the motor in position mode.')
    motor_driver.test_traj_pos()

    raw_input('Press enter to drive the motor in velocity mode.')
    motor_driver.test_traj_vel()

    raw_input('Press enter to drive the motor in a specific trajectory.')
    motor_driver.test_traj()

    print('That concludes this demonstration. Feel free to look at the code that makes up this demonstration.')

    print("When you're done, call over a lab instructor or TA to receive a check off.")