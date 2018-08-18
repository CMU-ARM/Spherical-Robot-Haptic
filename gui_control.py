#!/usr/bin/python3

from sphero_sprk.sphero import Sphero
import time
import datetime
import threading
import copy
import string
import numpy as np
import matplotlib.pyplot as plt

import tkinter as tk
from tkinter import font
import yaml
from SpheroLooper import SpheroLooper

import ast
import csv

BREAK = Sphero.RAW_MOTOR_MODE_BRAKE
FORWARD = Sphero.RAW_MOTOR_MODE_FORWARD
REVERSE = Sphero.RAW_MOTOR_MODE_REVERSE


orb = Sphero("fd:f7:ad:bd:d1:5c")
orb.connect()
looper = SpheroLooper(orb, 10)


#run a given command sequence where each value is seperate by
#; for number and , for side of motor
def run_text_cmd(num,check_var,text_box):
    text = text_box.get("1.0",tk.END)
    text = text.split(';')
    cmd = []
    repeat_flag = check_var.get()
    for arr in text:
        tuple_val = arr.split(',')
        cmd.append((int(tuple_val[0]),int(tuple_val[1])))
    max_times = 100 if repeat_flag == 1 else 1
    looper.unbalanced_seq(cmd,5,10,max_times)

#This generates the left and right direction pointers
def run_common_cmd(dir_, num):
    #seq = np.array([150,200,235,255,-255,-235,-225,-200,-175,-150,-125,-100,-50,0])
    #seq = np.array([200,-100,-50,0,0,0])
    #seq = np.array([255,-255,-150,-100,0,0])
    seq = np.array([255,-150,-100,0,0])
    #seq = np.array([250,-50,0,0,0])
    for i in range(0, num):
        seq = np.delete(seq,-1)
    if(dir_ == 'left'):
        seq = seq * -1
    looper.sequence_pulsing(seq,10)

def run_line_cmd(dir_, num):
    roll = [(-80,120),(-100,80),(-80,100),(200,-160)]
    #roll = [(-50,150),(-50,150)]
    #roll = [(-25,100),(-25,100)]
    #roll = [(25,-150),(25,-150)]
    #roll = [(25,-150),(25,-150)]
    #roll = [(65,-55),(55,-65)]
    looper.unbalanced_seq(roll,60,4)

dir_map = [
    315,0,45,270,0,90,225,180,135
]
dir_word_map = [
    "NW","Forward","NE","Left","Center","Right","SW","Backwards","SE"
]

class Window2():
    
    def __init__(self, master):
        self.frame = tk.Frame(master)
        self.customFont = font.Font(family="Helvetica", size=150)
        self.text = tk.Label(self.frame, font=self.customFont)
        self.change_text("Ready")
        self.text.pack()
        self.frame.pack()

        self._next_text = ""

    def change_text(self, text):
        self.text.config(text=text)
        self.text.update_idletasks()
        #self.text.delete('1.0',tk.END)
        #self.text.insert(tk.INSERT,"{}".format(text))

    def change_wait_text(self, text):
        self._next_text  = text
    
    def show_wait_text(self):
        if self._next_text != "":
            self.change_text(self._next_text)
            self._next_text = ""

class Window(tk.Frame):
    def __init__(self, master=None):
        tk.Frame.__init__(self, master)
        self.master = master
        self._frame = master
        self.populate_window()
        self.pack()

        self.newWindow = tk.Toplevel(master)
        self.text_window = Window2(self.newWindow)
        master.bind("<Return>", self.return_event)
        self._flat_set_flag = False
        self._encoding_start_flag = False
        self._dir_start_time = time.time()
        self._dir_heading = -1
        self._dir_flag = False

        self._log_file_name = "{}.log".format(time.strftime("%m-%d-%H-%M-%S"))
        self._log_file = open(self._log_file_name,'w')


    def return_event(self, event):

        end_time = time.time()
        self.text_window.show_wait_text()

        #record the information if needed
        if self._encoding_start_flag:
            #end_time = time.time()
            start_time = self._encoding_start_time
            name = self._current_enconding
            eval_flag = self.eval_check.get()
            learn_flag = self.learning_check.get()
            self._log_file.write("{},{},{},{},{},{}\n".format(name, start_time, end_time,end_time-start_time,learn_flag,eval_flag))
            self._log_file.flush()
            self._encoding_start_flag = False 
            #also run the return to 0 event
            self.return_to_forward()
            #self._frame['bg'] = 'white'
            #self._frame.update_idletasks()
            self.update_status("Done")
            #self._frame.config(Background="white")

        if self._dir_flag:
            self._dir_flag = False
            #end_time = time.time()
            start_time = self._dir_start_time
            heading = self._dir_heading
            eval_flag = self.eval_check.get()
            learn_flag = self.learning_check.get()
            self._log_file.write("{},{},{},{},{},{}\n".format(heading,start_time, end_time,end_time-start_time,learn_flag,eval_flag))
            self._log_file.flush()
            #self._frame['bg'] = 'white'
            #self._frame.update_idletasks()
            self.update_status("Done")
            
            
            #self._frame.config(Background="white")

        #print("In return event!")
        if self._flat_set_flag:
            self._flat_set_flag = False
            self._run_flat_set()

        if self.eval_check.get() == 0 and self.learning_check.get() == 0:
            self.text_window.change_text("Ready")



    def _toggle_control_loop(self,flag):
        if(flag):
            looper.start_control_loop()
        else:
            looper.stop_control_loop()
        #self.status_label["text"] = "control loop:{}".format("on" if flag else "off")

    def update_status(self, text):
        self.status_label["text"] = text

    def stabalize(self):
        heading = int(self.heading_text.get("1.0",tk.END))
        looper.stabilization(heading)
        #looper.calibrate(heading)

    def down_back(self):
        looper.up_down_back()

    def direction_cmd(self, num):
        print(num)
        heading = dir_map[num]
        self._dir_heading = heading
        word_name = dir_word_map[num]
        if self.learning_check.get() == 1:
            self.text_window.change_text(word_name)
        if self.eval_check.get() == 1:
            self.text_window.change_wait_text(word_name)
        self.heading_text.delete('1.0',tk.END)
        self.heading_text.insert(tk.INSERT,"{}".format(heading))
        #move to heading first

        looper.move_to_heading(heading)

    def return_to_forward(self):
        looper.move_to_heading(180)
        looper.move_to_heading(0)
        self.heading_text.delete('1.0',tk.END)
        self.heading_text.insert(tk.INSERT,"{}".format(0))

    def run_move_cmd(self):

        if self.learning_check.get() == 0:
            self.text_window.change_text("Running...")

        parsed_seq = [(200,195),(-100,-100),(-50,-50),(-50,-50),(0,0),(0,0),(0,0),(0,0)]

        looper.unbalanced_seq(parsed_seq,30,5,1)
        looper.wait_for_unbalanced_seq()
        #change heading
        heading = int(self.heading_text.get("1.0",tk.END))
        looper.move_to_heading(heading)
        #change run it again
        looper.unbalanced_seq(parsed_seq,30,5,1)
        looper.wait_for_unbalanced_seq()
        self._dir_flag = True
        self._encoding_start_flag = False
        self._dir_start_time = time.time()  
        # self._frame['bg'] = 'red'
        # self._frame.update_idletasks()
        self.update_status("SAVEEEEEEEEE")
        #self._frame.config(Background="red")      



    def run_encoding_cmd(self, name, seqs,max_times,flat_set=False,show_name=True,heading_offset=0):
        
        self._current_enconding = name
        self._encoding_start_time = time.time()
        self._encoding_start_flag = True
        self._dir_flag = False

        if show_name:
            if self.learning_check.get() == 1:
                self.text_window.change_text(name)
            else:
                self.text_window.change_text("Running...")
        #parse sequence
        parsed_seq = []
        for s in seqs:
            parsed_seq.append(ast.literal_eval(s))

        looper.unbalanced_seq(parsed_seq,30,5,max_times)
        looper.wait_for_unbalanced_seq()
        #time.sleep(0.5)
        #wait
        self._flat_set_flag = flat_set
        if show_name and self.eval_check.get() == 1:
            self.text_window.change_wait_text(name) 

        looper.add_heading_offset(heading_offset)
        #self._frame.config(Background="white")
        self.update_status("SAVEEEEEEEEE")
        
        # self._frame['bg'] = 'red'
        # self._frame.update_idletasks()
        
        #self._stored_offset += heading_offset


    def _run_flat_set(self):
        looper.up_down_back()
        heading = int(self.heading_text.get("1.0",tk.END))
        #print(heading)
        looper.reset_heading(heading)

        #looper.stabilization_fast(heading)

    def _manual_cal(self):
        looper.up_down_back()        
        looper.reset_heading(0)
        self.heading_text.delete('1.0',tk.END)
        self.heading_text.insert(tk.INSERT,"{}".format(0))  

    def calibration_action(self, type):
        self.text_window.change_text("Calibrating...")
        if type == 0:
            looper.push_calibration_action()
        elif type == 1:
            looper.movement_calibration_action()
        elif type == 2:
            looper.hand_calibration_action()
        self.text_window.change_text("Ready")
        self.heading_text.delete('1.0',tk.END)
        self.heading_text.insert(tk.INSERT,"{}".format(0))            

    def build_basic_control(self, frame):
        self.start_btn = tk.Button(frame)
        self.start_btn["text"] = "Start"
        self.start_btn["command"] = lambda: self._toggle_control_loop(True) 
        self.start_btn.pack(side=tk.LEFT)

        self.stop_btn = tk.Button(frame)
        self.stop_btn["text"] = "stop"
        self.stop_btn["command"] = lambda: self._toggle_control_loop(False) 
        self.stop_btn.pack(side=tk.LEFT)

        self.zero_btn = tk.Button(frame)
        self.zero_btn["text"] = "Zero"
        self.zero_btn["command"] = looper.break_control_loop
        self.zero_btn.pack(side=tk.LEFT)

        self.heading_text = tk.Text(frame)
        self.heading_text.insert(tk.INSERT,"0")
        self.heading_text["width"] = 3
        self.heading_text["height"] = 1
        self.heading_text.pack(side=tk.LEFT)

        self.stab_btn = tk.Button(frame)
        self.stab_btn["text"] = "Stabalize"
        self.stab_btn["command"] = self.stabalize
        self.stab_btn.pack(side=tk.LEFT)     

        self.manual_btn = tk.Button(frame)
        self.manual_btn["text"] = "Manual Calibration"
        self.manual_btn["command"] = self._manual_cal
        self.manual_btn.pack(side=tk.LEFT)

        self.return_btn = tk.Button(frame)
        self.return_btn["text"] = "Return"
        self.return_btn["command"] = self.return_to_forward
        self.return_btn.pack(side=tk.LEFT)

        # self.push_calibrate_btn = tk.Button(frame)
        # self.push_calibrate_btn["text"] = "Push Calibrate"
        # self.push_calibrate_btn["command"] = lambda type=0: self.calibration_action(type)
        # self.push_calibrate_btn.pack(side=tk.LEFT)   

        # self.move_calibrate_btn = tk.Button(frame)
        # self.move_calibrate_btn["text"] = "Move Calibrate"
        # self.move_calibrate_btn["command"] = lambda type=1: self.calibration_action(type)
        # self.move_calibrate_btn.pack(side=tk.LEFT)  
  
        self.hand_calibrate_btn = tk.Button(frame)
        self.hand_calibrate_btn["text"] = "Hand Calibrate"
        self.hand_calibrate_btn["command"] = lambda type=2: self.calibration_action(type)
        self.hand_calibrate_btn.pack(side=tk.LEFT)  

        self.down_back_btn = tk.Button(frame)
        self.down_back_btn["text"] = "Down Back"
        self.down_back_btn["command"] = self.down_back
        self.down_back_btn.pack(side=tk.LEFT)

        self.text_ready_btn = tk.Button(frame)
        self.text_ready_btn["text"] = "Ready Screen"
        self.text_ready_btn["command"] = lambda text="Ready":self.text_window.change_text(text)
        self.text_ready_btn.pack(side=tk.LEFT)

        self.learning_check = tk.IntVar()
        self.learning_check_btn = tk.Checkbutton(frame,
            text="LEARN",
            variable=self.learning_check
        )
        self.learning_check_btn.pack(side=tk.LEFT)

        self.eval_check = tk.IntVar()
        self.eval_check_bx = tk.Checkbutton(frame,
            text="EVAL",
            variable=self.eval_check
        )
        self.eval_check_bx.pack(side=tk.LEFT)

        self.status_label = tk.Label(frame)
        self.status_label["text"] = "Not Running"
        self.status_label.pack(side=tk.LEFT)

    def build_slider(self, frame):
        self._power_slider = tk.Scale(frame, from_= 50, to=250, resolution=50)
        self._power_slider = tk.Scale(frame, from_= 50, to=250, resolution=50)
        self._power_slider.grid(row=1,column=1)
        self._dir_opt = tk.Scale(frame, from_=0, to=1)
        self._dir_opt.grid(row=1,column=2)
        self._run_btn = tk.Button(frame)
        self._run_btn["text"] = "Run"
        self._run_btn["command"] = self._slider_opt
        self._run_btn.grid(row=1, column=3)

    def _slider_opt(self):
        value  = int(self._power_slider.get())
        main_dir = value
        #main_dir = np.min([int(value * 0.75),255])
        #bottom_dir = np.min([int(value * 0.25),255])

        #main_dir = np.min([int(value * 0.75),255])
        #bottom_dir = np.min([int(value * 0.25),255])

        bottom_dir = 50
        seq = np.array([main_dir,-1*bottom_dir,0,0,0])
        if(self._dir_opt.get() == 1):
            seq = seq * -1

        looper.sequence_pulsing(seq.tolist(),10) #right slow
        pass

    def populate_window(self):

        basic_grid = tk.Frame(self)
        self.build_basic_control(basic_grid)
        basic_grid.pack(side=tk.TOP)

        command_frame = tk.Frame(self)
        self.create_command_grid(5,command_frame)
        command_frame.pack()

        common_grid = tk.Frame(self)
        self.create_common_grid(common_grid)
        common_grid.pack(side=tk.LEFT)

        direction_grid = tk.Frame(self)
        self.create_direction_controls(direction_grid)
        direction_grid.pack(side=tk.LEFT)

        symbol_grid = tk.Frame(self)
        self.create_encoding_controls(symbol_grid)
        symbol_grid.pack(side=tk.LEFT)

        slider_grid = tk.Frame(self)
        self.build_slider(slider_grid)
        slider_grid.pack(side=tk.LEFT)

        fast_grid = tk.Frame(self)
        self.build_fast_heading(fast_grid)
        fast_grid.pack(side=tk.LEFT)     

    def fast_heading(self, heading):
        self.heading_text.delete('1.0',tk.END)
        self.heading_text.insert(tk.INSERT,str(heading))
        looper.stabilization(heading)


    def build_fast_heading(self, frame):
        angles = [0,90,180,270,360]
        for i,angle in enumerate(angles):
            btn = tk.Button(frame)
            btn["text"] = str(angle)
            btn["command"] = lambda angle=angle:self.fast_heading(angle)

            btn.grid(row=i,column=1)

    def create_common_grid(self, frame):
        label = tk.Label(frame,text="Common Controls")
        label.grid(row=0,column=0)
        directions = ["left","right"]
        for x, dir_ in enumerate(directions):

            ["left","right"]
            for i in range(0,3):
                mv_btn = tk.Button(frame)
                mv_btn["text"] = "{} {}".format(dir_, i)
                mv_btn["command"] = lambda num=i,dir_=dir_: run_common_cmd(dir_,num)
                mv_btn.grid(row=i+1,column=x)



    def create_direction_controls(self, frame):
        label = tk.Label(frame,text="Direction Controls")
        label.grid(row=0,columnspan=3)
        mapping = {1:"<Up>",3:"<Left>",5:"<Right>",7:"<Down>"}
        for x in range(0,9):
            btn = tk.Button(frame)
            btn["text"] = str(x)
            func = lambda num=x: self.direction_cmd(num)
            btn["command"] = func
            btn.grid(row=1+(int(x/3)),column=x%3)
            if x in [1,3,5,7]:
                self.master.bind(mapping[x],lambda event,num=x: self.direction_cmd(num))
        move_btn = tk.Button(frame)
        btn["text"] = "MOVE"
        func = self.run_move_cmd
        btn["command"] = func
        btn.grid(row=4,columnspan=3)

    def create_line_controls(self, frame):
        label = tk.Label(frame,text="Line Controls")
        label.grid(row=0,column=0)
        directions = ["left","right"]
        for x, dir_ in enumerate(directions):

            ["left","right"]
            for i in range(0,2):
                mv_btn = tk.Button(frame)
                mv_btn["text"] = "{} {}".format(dir_, i)
                mv_btn["command"] = lambda num=i,dir_=dir_: run_line_cmd(dir_,num)
                mv_btn.grid(row=i+1,column=x)        

    def create_encoding_controls(self, frame):
        label = tk.Label(frame,text="Symbol Controls")
        label.grid(row=0,column=0)
        with open("encoding.yaml",'r') as enc:
            encoding = yaml.load(enc)
            for i,symbol in enumerate(encoding):
                sym_btn = tk.Button(frame)
                symbol = dict(symbol)
                max_times = symbol.get('max_times',0)
                heading_offset = symbol.get('heading_offset',0)
                flat_set = symbol.get('flat_set',False)
                show_name = symbol.get('show_name',True)
                sym_btn["text"] = symbol["name"]
                run_func = lambda heading_offset=heading_offset,show_name=show_name,flat_set=flat_set,maxt=max_times,name=symbol["name"],end=symbol["encode"]: self.run_encoding_cmd(name,end,maxt,flat_set,show_name,heading_offset)
                sym_btn["command"] = run_func
                sym_btn.grid(row=int(i/2)+1,column=int(i%2),sticky='W')

                #bind if they are single characters
                if len(symbol["name"]) == 1:
                    sym_name = symbol["name"].lower()
                    self.master.bind(sym_name,run_func)
                    self.master.bind(string.ascii_lowercase.index(sym_name) + 1,run_func)


    def create_command_grid(self, num,frame):
        
        self._command_text_list = []
        for i in range(0,num):
            text_box = tk.Text(frame, height=1)
            text_box.grid(row=i,column=0)
            self._command_text_list.append(text_box)

            check_var = tk.IntVar()
            check_btn = tk.Checkbutton(frame,
                text="Repeat",
                variable=check_var
            )
            check_btn.grid(row=i, column=1)        

            cmd_btn = tk.Button(frame)
            cmd_btn["text"] = "Run {}".format(i+1)
            cmd_btn["command"] = lambda num=i,var=check_var: run_text_cmd(num,var,self._command_text_list[num])
            cmd_btn.grid(row=i,column=2)


def close_function(root):
    looper.stop_control_loop()
    looper.breaking()
    orb.sleep()
    root.destroy()

root = tk.Tk()
root.protocol("WM_DELETE_WINDOW",lambda root=root:close_function(root))
app = Window(master=root)

app._toggle_control_loop(True)
app.mainloop()
