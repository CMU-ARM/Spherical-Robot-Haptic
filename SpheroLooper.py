#!/usr/bin/python3

from sphero_sprk.sphero import Sphero
import time
import threading
import copy
import numpy as np
import matplotlib.pyplot as plt

import tkinter as tk
from Tools import Tools


BREAK = Sphero.RAW_MOTOR_MODE_BRAKE
FORWARD = Sphero.RAW_MOTOR_MODE_FORWARD
REVERSE = Sphero.RAW_MOTOR_MODE_REVERSE

class SpheroLooper(object):

    def _imu_callback(self,data):
        time_now = time.time()
        self._last_imu = data
        #print("IMU Hz:{}".format(1/(time_now - self._imu_time)))
        self._imu_time = time_now

    def _accel_callback(self,data):
        time_now = time.time()     
        if self._last_accel is None:
            self._last_accel = data
        else:
            alpha = 0.5
            self._last_accel['x'] = Tools.exponential_moving_average(self._last_accel['x'],data['x'],alpha) 
            self._last_accel['y'] = Tools.exponential_moving_average(self._last_accel['y'],data['y'],alpha) 
            self._last_accel['z'] = Tools.exponential_moving_average(self._last_accel['z'],data['z'],alpha)  
        
        self._accel_dt = time_now - self._accel_time
        #print("Accel Hz:{}".format(1/(time_now - self._accel_time)))
        self._accel_time = time_now

    def __init__(self, orb, rate):
        self._orb = orb
        self._rate = rate
        self._dir = [Sphero.RAW_MOTOR_MODE_FORWARD, Sphero.RAW_MOTOR_MODE_REVERSE]
        self._power = [0,0]
        self._pause_flag = False
        self._stop_flag = False
        self._control_thread = None
        self._control_rate = 20 #20Hz is the maximum we can send to the system, seemed to be a limitation 
        self._last_imu = None
        self._last_accel = None
        self._accel_dt = 0
        self._seq_flag = False
        self._seq_thread = None

        self._fo_heading = 0 #heading that is faked

        self._imu_time = time.time()
        self._orb.start_IMU_callback(20,self._imu_callback)
        self._accel_time = time.time()
        self._orb.start_accel_callback(20,self._accel_callback)

    def change_power(self, new_power_setting):
        for i, power in enumerate(new_power_setting):
            if power == 0:
                self._dir[i] = Sphero.RAW_MOTOR_MODE_BRAKE
                self._power[i] = 0
            elif power < 0:
                self._dir[i] = Sphero.RAW_MOTOR_MODE_REVERSE
                self._power[i] = -1 * power
            else:
                self._dir[i] = Sphero.RAW_MOTOR_MODE_FORWARD
                self._power[i] = power

    def start_control_loop(self):
        self._stop_flag = False
        self._control_thread = threading.Thread(target=self._control_loop)
        self._control_thread.start()
        print("control loop started")

    def stop_control_loop(self):
        self._stop_flag = True
        self._control_thread.join()
        self.breaking()
        print("control loop stop")

    def breaking(self):
        self._orb.set_raw_motor_values(Sphero.RAW_MOTOR_MODE_BRAKE,0,Sphero.RAW_MOTOR_MODE_BRAKE,0)

    def break_control_loop(self):
        #stop the sequence looper 
        self._stop_seq_loop()
        #break the control loop
        self._dir[0] = Sphero.RAW_MOTOR_MODE_BRAKE
        self._dir[1] = Sphero.RAW_MOTOR_MODE_BRAKE
        self._power[0] = 0
        self._power[1] = 0
        print(self._power)

    def up_down_back(self):
        while np.abs(self._last_imu["pitch"]) > 5:
            start_time = time.time()
            direction = Sphero.RAW_MOTOR_MODE_FORWARD if self._last_imu["pitch"] < 0 else Sphero.RAW_MOTOR_MODE_REVERSE
            pwd_val = 35#np.max([35,np.min([40,np.abs(self._last_imu["pitch"]) + 10])])
            print(pwd_val)

            self._dir[0] = direction
            self._dir[1] = direction
            self._power[0] = pwd_val
            self._power[1] = pwd_val
            
            sleep_time = 1/self._rate - (time.time() - start_time)
            if(sleep_time > 0):
                time.sleep(sleep_time)

            #self._orb.set_raw_motor_values(direction,pwd_val,direction,pwd_val)
        print("DONE")
        self.break_control_loop()
        #self.breaking()

    def stabilization(self, final_heading=0):
        #first pause the control loop
        self._pause_flag = True
        #run the stabalization code
        self._orb.set_stabilization(True)
        #we roll a bit because it shown to help stabalization
        print((180-final_heading)%359)
        self._orb.roll(0,(final_heading+180)%359)
        time.sleep(1)   
        #print(final_heading)
        self._orb.roll(0,final_heading)
        #wait for a 1 second
        time.sleep(1)
        #self._orb.set_heading(0)
        self._pause_flag = False

    def acknowledgement_vibe(self):
        #vibe to show that we get it
        self.change_power([70,-70])
        time.sleep(0.1)
        self.change_power([-70,70])
        time.sleep(0.1)
        self.change_power([70,-70])
        time.sleep(0.1)
        self.change_power([-70,70])
        time.sleep(0.1)
        self.change_power([0,0])
        time.sleep(0.2)

    def push_calibration_action(self):
        #action sequence that help calibrating the yaw by acclerating/push in certain direction
        #store the current diff:
        init_x = self._last_accel['x']
        init_y = self._last_accel['y']
        
        limit = 2000
        #wait until either of them pass the limit
        while True:
            if np.abs(self._last_accel['x'] - init_x) > limit:
                break
            if np.abs( self._last_accel['y'] - init_y) > limit:
                break
        val_x = self._last_accel['x']
        val_y = self._last_accel['y']
        #send to backend
        self._accel_based_calibration_backend(val_x, val_y)

    def movement_calibration_action(self):
        #action sequence that help calibrating the yaw by moving in certain direction
        #store the current diff:

        # vel_x = 0#self._last_accel['x']
        # vel_y = 0#self._last_accel['y']

        init_x = self._last_accel['x']
        init_y = self._last_accel['y']
        
        limit = 1000
        val_x = 0
        val_y = 0 
        while True:    
            #calculate the travelled distance
            #store the difference between accelerations
            val_x += (self._last_accel['x'] - init_x)
            init_x = self._last_accel['x']
            val_y += (self._last_accel['y'] - init_y)
            init_y = self._last_accel['y']

            if(np.abs(val_x) > limit or np.abs(val_y) > limit):
                break

        self._accel_based_calibration_backend(val_x, val_y)

    def _accel_based_calibration_backend(self, val_x, val_y):
        #normalize vector
        vec = [val_x, val_y,0]
        print(vec)
        vec = vec/np.linalg.norm(vec)
        #figure out how much to turn back
        base_vec = np.array([0,-1,0]) #because we are trying to align the negative y-axis to the front
        print(vec)
        roll_val = np.rad2deg(Tools.get_2D_rotation(vec, base_vec))
        roll_val = int(roll_val)
        print(roll_val)
        if(roll_val < 0):
            #roll_val = roll_val*-1
            roll_val = 360 + roll_val
        print(roll_val)
        #print("accel x:{}, accel y:{}".format(val_x, val_y))
        #send the calibration to the backend
        self._calibration_backend(roll_val)

    def add_heading_offset(self, offsets):
        self._fo_heading += offsets
        print(self._fo_heading)


    def hand_calibration_action(self):
        #action sequence that help with the calibration

        #first store the current diff
        init_roll = self._last_imu['roll']
        init_pitch = self._last_imu['pitch']

        limit = 20
        #wait until either of them is pass the limit
        while True:
            if np.abs(self._last_imu['roll'] - init_roll) > limit:
                break
            if np.abs(self._last_imu['pitch'] - init_pitch) > limit:
                break
        #save those two values
        roll_x = self._last_imu['roll']
        pitch_y = self._last_imu['pitch']

        #calculate how much to rotate from current plane
        point, normal = Tools.create_plane_from_roll_pitch(roll_x,pitch_y)
        vec = Tools.project_to_plane(normal,np.array([0,0,1]))
        vec = vec/np.linalg.norm(vec)
        base_vec = np.array([1,0,0])
        roll_val = np.rad2deg(Tools.get_2D_rotation(base_vec, vec))
        roll_val = int(roll_val)
        if(roll_val < 0):
            roll_val = 360 + roll_val
        #print("roll:{}, pitch:{}".format(roll_x, pitch_y))
        self._calibration_backend(roll_val)

    def _calibration_backend(self, roll_val):
        #first do the acknowledge vibe
        self.acknowledgement_vibe()
        #print roll value for debugging
        print("roll_val:{}".format(roll_val))
        #pause control loop
        self._pause_flag = True
        #start stabalization
        self._orb.set_stabilization(True)
        self._orb.set_heading(0)
        self._orb.roll(0,roll_val)
        time.sleep(0.5)
        #self.stabilization()
        self._orb.set_heading(0)
        self._fo_heading = 0
        self._orb.set_stabilization(False)        
        #resume control loop
        self._pause_flag = False

    def reset_heading(self, cur_heading):
        #first pause the control loop
        self._pause_flag = True
        #run the stabalization code
        self._orb.set_stabilization(True)
        #set the current heading as 0
        self._orb.set_heading(0,True)
        #roll to the 0 possition
        # roll_heading = 360 - cur_heading
        # self._orb.roll(0, roll_heading)
        # time.sleep(0.5)
        # self._orb.set_heading(0,True)    
        # self._orb.roll(0, cur_heading)    
        # time.sleep(0.5)

        #calculate the difference from the current position to the expected heading
        self._fo_heading = cur_heading

        self._orb.set_stabilization(False)
        self._pause_flag = False

    def move_to_heading(self, heading):
        #move to the heading
        self._pause_flag = True
        self._orb.set_stabilization(True)

        #old code
        #self._orb.roll(0,heading)
        #time.sleep(1)

        #instead of 
        new_heading = int(np.round(heading - self._fo_heading))
        while new_heading < 0:
            new_heading = new_heading + 360
        while new_heading >= 360:
            new_heading = new_heading - 360

        print("True heading:{}".format(new_heading))    
        self._orb.roll(0,new_heading)
        time.sleep(0.5)


        self._orb.set_stabilization(False)
        self._pause_flag = False

    def stabilization_fast(self, final_heading=0):
        #first pause the control loop
        self._pause_flag = True
        #run the stabalization code
        self._orb.set_stabilization(True)
        #we roll a bit because it shown to help stabalization
        self._orb.roll(0,final_heading)
        #wait for a 1 second
        time.sleep(1)
        #self._orb.set_heading(0)
        self._pause_flag = False

    def calibrate(self, new_heading):

        #first pause the control loop
        self._pause_flag = True
        #run the stabalization code
        self._orb.set_stabilization(True)
        self._orb.roll(0,(new_heading+180)%359)
        time.sleep(1)
        self._orb.roll(0,new_heading)
        time.sleep(1)
        self._orb.set_heading(0)
        self._pause_flag = False


    def _control_loop(self):
        while(not self._stop_flag):
            start_time = time.time()
            if(not self._pause_flag):
                self._orb.set_raw_motor_values(self._dir[0],self._power[0],self._dir[1],self._power[1])
            action_time = time.time() - start_time
            sleep_time = 1/self._control_rate - action_time
            #print("control Hz:{}".format(1/action_time))
            if(sleep_time > 0):
                time.sleep(sleep_time)

    def _sequence_loop(self,seq_tuple,total_time,max_times=0):
        total_start_time = time.time()
        index = 0
        counter = 0
        #print("max:{}".format(max_times))
        while(self._seq_flag and ( (max_times == 0 or counter < max_times) and (time.time() - total_start_time < total_time))):
            start_time = time.time()

            seq = seq_tuple[index]
            self.change_power([seq[0], seq[1]])
            sleep_time = 1/self._rate - (time.time() - start_time)
            if(sleep_time > 0):
                time.sleep(sleep_time)
            index = (index + 1)%np.size(seq_tuple,0)
            if(index == 0):
                counter += 1
               #print("update{}".format(counter))

        #stop the thing from moving
        self._dir[0] = Sphero.RAW_MOTOR_MODE_BRAKE
        self._dir[1] = Sphero.RAW_MOTOR_MODE_BRAKE
        self._power[0] = 0
        self._power[1] = 0        

    def _start_seq_loop(self, seq_tuple,total_time,max_times=5):
        if(self._seq_thread != None and self._seq_flag == True):
            self._stop_seq_loop()

        self._seq_flag = True
        self._seq_thread = threading.Thread(target=self._sequence_loop,args=(seq_tuple,total_time,max_times))
        self._seq_thread.start()
        print("sequence loop started:{}".format(seq_tuple))


    def _stop_seq_loop(self):
        if(self._seq_thread != None and self._seq_flag == True):
            self._seq_flag = False
            self._seq_thread.join()
            self._seq_thread = None
            print("sequence loop stopped")

    def sequence_pulsing(self, seq, total_time):
        seq_tuple = []
        #convert the sequence to the looping nature
        for s in seq:
            seq_tuple.append((s,-1*s))
        self._start_seq_loop(seq_tuple,total_time)


    def unbalanced_seq(self,seq_tuple, total_time, rate=-1, max_times=0):


        self._start_seq_loop(seq_tuple,total_time,max_times)
        # total_start_time = time.time()
        # index = 0

        # if(rate == -1):
        #     rate = self._rate

        # while(time.time() - total_start_time < total_time):
        #     print(index)
        #     start_time = time.time()
        #     seq = seq_tuple[index]
        #     self.change_power([seq[0], seq[1]])
        #     sleep_time = 1/rate - (time.time() - start_time)
        #     if(sleep_time > 0):
        #         time.sleep(sleep_time)
        #     index = (index + 1)%len(seq_tuple)

    def wait_for_unbalanced_seq(self,timeout=0):
        self._seq_thread.join()

    def manual_control(self, seq_list, total_time):
        #first stop the control loop
        self.stop_control_loop()
        total_start_time = time.time()
        index = 0
        while(time.time() - total_start_time < total_time):
            start_time = time.time()

            dir1, power1, dir2, power2 = seq_list[index]
            self._orb.set_raw_motor_values(dir1, power1, dir2, power2)

            sleep_time = 1/self._rate - (time.time() - start_time)
            if(sleep_time > 0):
                time.sleep(sleep_time)
            index = (index + 1)%len(seq_list)       

    def tick(self,interval,total_time):
        #first stop the control loop
        #self.stop_control_loop()
        self.change_power([0,0])
        total_start_time = time.time()
        while(time.time() - total_start_time < total_time):
            start_time = time.time()

            self.change_power([50,-50])
            time.sleep(0.1)
            self.change_power([-50,50])
            time.sleep(0.1)
            self.change_power([0,0])
            # self._orb.set_raw_motor_values(FORWARD, 100, REVERSE, 100)
            # time.sleep(200)
            # self._orb.set_raw_motor_values(REVERSE, 100, FORWARD, 100)
            # time.sleep(200)
            # self._orb.set_raw_motor_values(BREAK, 100, BREAK, 100)

            sleep_time = interval
            if(sleep_time > 0):
                time.sleep(sleep_time)


    def vibe(self,interval,total_time):
        #first stop the control loop
        #self.stop_control_loop()
        self.change_power([0,0])
        total_start_time = time.time()
        index = 0
        while(time.time() - total_start_time < total_time):
            start_time = time.time()
            if(index%2 == 0):
                self.change_power([50,-50])
            else:
                self.change_power([-50,50])

            time.sleep(0.1)
            self.change_power([0,0])
            # self._orb.set_raw_motor_values(FORWARD, 100, REVERSE, 100)
            # time.sleep(200)
            # self._orb.set_raw_motor_values(REVERSE, 100, FORWARD, 100)
            # time.sleep(200)
            # self._orb.set_raw_motor_values(BREAK, 100, BREAK, 100)
            index = (index +1)%2
            sleep_time = interval
            if(sleep_time > 0):
                time.sleep(sleep_time)      