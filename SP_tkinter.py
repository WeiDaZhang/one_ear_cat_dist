#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tues July 2 12:18:00 2024

@author: Dr. Vincent Lee
Bioptix Ltd.

SP_tkinter.py
Description: Tkinter GUI classes for GUI on local machine

"""

from tkinter import *
from PIL import Image, ImageTk

import time
import numpy as np
import cv2 as cv
import matplotlib.pyplot as plt

from image_window import Image_window
from image_process import Image_process
from threading import Thread

from skimage.restoration import denoise_tv_chambolle

from heka import Heka
from batch import Batch


class Window:

    def __init__(self, cam):

        self.pvcam_instance = cam

        self.window = Toplevel()

        self.window_label = Label(self.window)
        self.window_label.pack()

        thread = Thread(target = self.video_loop())
        thread.daemon = True
        thread.start()

        self.window.wm_protocol("WM_DELETE_WINDOW", self.stop_video)

    #####----------FUNCTIONS----------#####

    def video_loop(self):

        cap = self.pvcam_instance.get_frame()
        img = Image.fromarray(cap)
        photo_img = ImageTk.PhotoImage(image = img)

        self.window_label.image = photo_img
        self.window_label.configure(image = photo_img)

        self.window.after(10, self.video_loop)  #milliseconds

    def stop_video(self):
        self.window.destroy()


######################################################################################################################################################

class App(Tk):

    def __init__(self, slicescope, condenser, cam, patchstar_list):

        self.slicescope_instance = slicescope
        self.condenser_instance = condenser
        self.pvcam_instance = cam
        self.patchstar_instance_list = patchstar_list

        self.window = []

        self.k_centroid_list = []
        self.tip_in_focus_corners = []
        self.offset_x_pixels = []
        self.offset_y_pixels = []
        self.tip_angle = []

        self.target1 = np.zeros((3,), dtype = int)
        self.target2 = np.zeros((3,), dtype = int)


        self.creepZ = 0

        self.slicescope_coordinates = []
        self.patchstar_list_coordinates = []

        self.image_stack = []
        self.image_stack_coord = []

        #HEKA batch comm setting
        self.batch_flag = False

    ################################################################

        #Main menu

        #main_frame_color = "white"
        sidebar_color = "gray"
        submenu_color = "blue"

        #gui_width = 1800
        #gui_height = 1000
        #screen_dimensions = str(gui_width) + 'x' + str(gui_height)

        Tk.__init__(self)
        self.title("Bioptix")
        #self.geometry(screen_dimensions)
        self.resizable(0,0)
        self.bind('<Escape>', lambda e: self.quit())

        # Add menu bar in root window
        menuBar = Menu(self) #Top level menu item
        fileMenuItems = Menu(menuBar, tearoff = 0) #2nd level item (1st below menu item)
        fileMenuItems.add_command(label = 'Quit', command = self.quit) #add quit menu item to this fileMenuItems bar.
        menuBar.add_cascade(label = 'File', menu = fileMenuItems) #top level name and menu attached.
        self.configure(menu = menuBar)

        menu_border = 5

        #Sidebar
        #self.sidebar = Frame(self, bg = sidebar_color, width = 200, height = gui_height, relief = RAISED, borderwidth = menu_border)
        self.sidebar = Frame(self, bg = sidebar_color, relief = RAISED, borderwidth = menu_border)
        self.sidebar.pack(side = LEFT, fill = BOTH, expand = FALSE)

        #Logo on top in sidebar
        #self.logo = Frame(self.sidebar, bg = sidebar_color, width = 100, height = 100, relief = RAISED, borderwidth = menu_border)
        self.logo = Frame(self.sidebar, bg = sidebar_color, borderwidth = menu_border)
        self.logo.pack()
        logo_src = Image.open(r'C:\\Users\\Lab\\OneDrive - BioVision Group Limited\\Desktop\\Local smart patch folder\\bioptix_logo_192x49.png')
        logo_img = ImageTk.PhotoImage(logo_src)
        self.logo_label = Label(self.logo, image=logo_img)
        self.logo_label.pack()
        self.logo_label.image = logo_img  #Must reference image to show photoimage in tkinter and prevent Garbage Collection.

    ################################################################

        #Buttons

        btn_border = 5
        btn_width = 15
        btn_height = 1
        btn_font = ("Arial",10,"bold")

        #Camera Control - Buttons to move the camera and adjust the focus

        self.entry_step = 0
        self.entry_focus_step = 0
        self.entry_approach_step = 0
        self.entry_distance = 0
        self.entry_speed = 0

        self.submenu_frame1 = Frame(self.sidebar, bg = submenu_color, relief = RAISED, borderwidth = menu_border)
        self.submenu_frame1.pack()
        self.submenu_frame_label1 = Label(self.submenu_frame1, text = "CAMERA", font = ("Arial",15,"bold"), padx=50)
        self.submenu_frame_label1.pack()

        btn_video_hw = Button(self.submenu_frame1, text = "Video H/W", font = btn_font, width = btn_width, height = btn_height, command = self.video_hw, relief = RAISED, borderwidth = btn_border)
        btn_video_hw.pack()

        btn_video_sw = Button(self.submenu_frame1, text = "Video S/W", font = btn_font, width = btn_width, height = btn_height, command = self.video_sw, relief = RAISED, borderwidth = btn_border)
        btn_video_sw.pack()

    ################################################################

        #self.submenu_frame2 = Frame(self.sidebar, bg = submenu_color, relief = RAISED, borderwidth = menu_border)
        #self.submenu_frame2.pack()
        #self.submenu_frame_label2 = Label(self.submenu_frame2, text = "RE-CENTER", font = ("Arial",15,"bold"), padx=35)
        #self.submenu_frame_label2.pack()

        #btn_slicescope_reset_coord_system = Button(self.submenu_frame2, text = "Slicescope", font = btn_font, width = btn_width, height = btn_height, command = self.slicescope_reset_coord_system, relief = RAISED, borderwidth = btn_border)
        #btn_slicescope_reset_coord_system.pack()
        #btn_patchstar_reset_coord_system = Button(self.submenu_frame2, text = "Patchstar", font = btn_font, width = btn_width, height = btn_height, command = self.patchstar_reset_coord_system, relief = RAISED, borderwidth = btn_border)
        #btn_patchstar_reset_coord_system.pack()

    ################################################################

        self.submenu_frame3 = Frame(self.sidebar, bg = submenu_color, relief = RAISED, borderwidth = menu_border)
        self.submenu_frame3.pack()
        self.submenu_frame_label3 = Label(self.submenu_frame3, text = "HEKA", font = ("Arial",15,"bold"), padx=65)
        self.submenu_frame_label3.pack()

        btn_batch_iv_monitor = Button(self.submenu_frame3, text = "Batch I-V", font = btn_font, width = btn_width, height = btn_height, command = self.heka_iv_monitor, relief = RAISED, borderwidth = btn_border)
        btn_batch_iv_monitor.pack()

        btn_batch_loop_monitor = Button(self.submenu_frame3, text = "Batch Loop", font = btn_font, width = btn_width, height = btn_height, command = self.heka_loop_monitor, relief = RAISED, borderwidth = btn_border)
        btn_batch_loop_monitor.pack()

        btn_batch_Rpip_monitor = Button(self.submenu_frame3, text = "Batch Rpip", font = btn_font, width = btn_width, height = btn_height, command = self.heka_Rpip_monitor, relief = RAISED, borderwidth = btn_border)
        btn_batch_Rpip_monitor.pack()

        btn_stop_batch = Button(self.submenu_frame3, text = "Stop Batch", font = btn_font, width = btn_width, height = btn_height, command = self.stop_batch, relief = RAISED, borderwidth = btn_border)
        btn_stop_batch.pack()

    ################################################################

        self.submenu_frame4 = Frame(self.sidebar, bg = submenu_color, relief = RAISED, borderwidth = menu_border)
        self.submenu_frame4.pack()
        self.submenu_frame_label4 = Label(self.submenu_frame4, text = "LOW MAG", font = ("Arial",15,"bold"), padx=43)
        self.submenu_frame_label4.pack()

        self.distance_label = Label(self.submenu_frame4, text = "Distance (um)", font = btn_font)
        self.distance_label.pack()
        self.distance_scale_txt = Entry(self.submenu_frame4, width = 15)
        self.distance_scale_txt.pack()

        btn_distance_set_scale = Button(self.submenu_frame4, text = "Set", command = lambda: self.get_distance(self.distance_scale_txt), font = btn_font)
        btn_distance_set_scale.pack()

        self.speed_label = Label(self.submenu_frame4, text = "Speed (um/s)", font = btn_font)
        self.speed_label.pack()
        self.speed_scale_txt = Entry(self.submenu_frame4, width = 15)
        self.speed_scale_txt.pack()

        btn_speed_set_scale = Button(self.submenu_frame4, text = "Set", command = lambda: self.get_speed(self.speed_scale_txt), font = btn_font)
        btn_speed_set_scale.pack()

        #btn_creep_a = Button(self.submenu_frame4, text = "Creep A", font = btn_font, width = btn_width, height = btn_height, command = lambda: self.patchstar_instance_list[0].creepA(int(self.entry_distance),int(self.entry_speed)), relief = RAISED, borderwidth = btn_border)
        #btn_creep_a.pack()

        #btn_creep_get_frame = Button(self.submenu_frame4, text = "Creep Get Frame", font = btn_font, width = btn_width, height = btn_height, command = lambda: self.creep_get_frame(int(self.entry_distance),int(self.entry_speed)), relief = RAISED, borderwidth = btn_border)
        #btn_creep_get_frame.pack()

        btn_low_mag_find_tip = Button(self.submenu_frame4, text = "Find tip", font = btn_font, width = btn_width, height = btn_height, command = lambda: self.low_mag_find_tip(int(self.entry_distance),int(self.entry_speed)), relief = RAISED, borderwidth = btn_border)
        btn_low_mag_find_tip.pack()


#####----------FUNCTIONS----------#####

    def get_distance(self, distance_scale_txt):

        self.entry_distance = int(distance_scale_txt.get())

    def get_speed(self, speed_scale_txt):

        self.entry_speed = speed_scale_txt.get()

        if int(self.entry_speed) < 1_00:
            self.entry_speed = 0
        else:
            self.entry_speed = int(self.entry_speed)

    def video_hw(self):

        self.window = Window(self.pvcam_instance)

    def video_sw(self):

        self.window = Toplevel()

        self.window_label = Label(self.window)
        self.window_label.pack()

        self.window.wm_protocol("WM_DELETE_WINDOW", self.stop_video_sw)

        thread = Thread(target = self.video_sw_loop())
        thread.daemon = True
        thread.start()

    def video_sw_loop(self):

        img = Image.fromarray(self.pvcam_instance.frame_norm)
        photo_img = ImageTk.PhotoImage(image = img)

        self.window_label.image = photo_img
        self.window_label.configure(image = photo_img)

        self.window.after(10, self.video_sw_loop)  #milliseconds

    def stop_video_sw(self):

        self.window.destroy()

    def get_snapshot(self, label):

        cap = self.pvcam_instance.get_frame()  #See PVCAM for exp. time in milliseconds
        img = Image.fromarray(cap)
        photo_img = ImageTk.PhotoImage(image = img)

        label.image = photo_img
        label.configure(image = photo_img)

        return cap

    def slicescope_reset_coord_system(self):

        #Save current coordinate as condenser origin
        self.condenser_instance.coordinates()
        self.condenser_instance.z_origin = self.condenser_instance.z

        #Move condenser out of the way
        self.condenser_instance.moveRelative(self.condenser_instance.z_origin,100000_00)

        #Run Slicescope re-centering
        self.slicescope_instance.reset_coord_system()

        #Move condenser back to original position
        self.condenser_instance.moveAbsolute(self.condenser_instance.z_origin)

    def patchstar_reset_coord_system(self):

        thread = [None] * len(self.patchstar_instance_list)

        #Simultaneously re-center the patchstars
        for idx in range(len(thread)):

            thread[idx] = Thread(target = self.patchstar_instance_list[idx].reset_coord_system)
            thread[idx].daemon = True
            thread[idx].start()

        #Join the threads after the patchstars have simultaneously been re-centered
        for idx in range(len(thread)):
            thread[idx].join()


    def update_coordinates(self):

        #Update current position of slicescope.
        self.slicescope_instance.coordinates()

        #Update deltas w.r.t. maximum limits of slicescope using current coordinates.
        self.slicescope_instance.deltas()

        for idx in range(len(self.patchstar_instance_list)):

            #Update current position of patchstars.
            self.patchstar_instance_list[idx].coordinates()

            #Update deltas w.r.t. maximum limits of patchstars using current coordinates.
            self.patchstar_instance_list[idx].deltas()


    def heka_iv_monitor(self):

        heka_proc = Heka()
        batch_proc = Batch()

        heka_proc.initialize()

        time.sleep(0.1)
        
        thread_batch = Thread(target = batch_proc.batch_comm())
        thread_batch.daemon =True
        thread_batch.start()

        self.batch_flag = True
        
        time.sleep(0.1)
        
        heka_proc.iv_monitor(self.batch_flag)

    def heka_loop_monitor(self):

        heka_proc = Heka()
        batch_proc = Batch()

        heka_proc.initialize()

        time.sleep(0.1)
        
        thread_batch = Thread(target = batch_proc.batch_comm())
        thread_batch.daemon = True
        thread_batch.start()

        self.batch_flag = True
        
        time.sleep(0.1)

        heka_proc.loop_monitor(self.batch_flag)

    def heka_Rpip_monitor(self):

        heka_proc = Heka()
        batch_proc = Batch()

        heka_proc.initialize()

        time.sleep(0.1)
        
        thread_batch = Thread(target = batch_proc.batch_comm())
        thread_batch.daemon = True
        thread_batch.start()

        self.batch_flag = True
        
        time.sleep(0.1)

        heka_proc.Rpip_monitor(self.batch_flag)

    def stop_batch(self):

        batch_proc = Batch()

        batch_proc.batch_comm()
        self.batch_flag = False

    def low_mag_find_tip(self, distance, speed):

        self.creep_get_frame(distance, speed)

        self.find_tip_harris_corner()


    def creep_get_frame(self, distance, speed):

        self.image_stack = []
        self.image_stack_coord = []

        duration = int(abs(distance)/speed)
        elapsed = 0

        self.slicescope_instance.creepZ(distance, speed)

        tic = time.time()

        cnt = 0
        while elapsed < duration:
            thread = Thread(target = self.slicescope_instance.coordinates)
            thread.daemon = True
            thread.start()
            self.image_stack.append(self.pvcam_instance.get_frame())
            thread.join()

            #cv.imshow('img stack',self.image_stack[-1])
            #cv.waitKey(10)

            print(f'Z = {self.slicescope_instance.z}')

            self.image_stack_coord.append(self.slicescope_instance.z)

            toc = time.time()
            #print(toc-tic)
            elapsed += toc-tic

            tic = toc
            cnt += 1

        self.image_stack.pop(0)
        self.image_stack_coord.pop(0)

        #plt.plot(self.image_stack_coord)
        #plt.show()

        #for idx in range(len(self.image_stack)):
        #    cv.imwrite('C:/Users/Lab/Images/creep sequence/' + str(idx) + '.bmp', self.image_stack[idx])

    def find_tip_harris_corner(self):

        img_proc = Image_process()

        #Convert the list of images into a numpy array. Dimensions are (Num frames, Height, Width)
        img_stack_array = np.array(self.image_stack)

        #Move the index for the number of images in array to the end. Dimensions are (Height, Width, Num frames)
        img_stack_array = np.moveaxis(img_stack_array, 0, -1)

        u = []

        for idx in range(img_stack_array.shape[2]):   #len(img_stack_array)

            #Use contour to determine if any part of the probe is in focus
            img_proc.load_frame(img_stack_array[:,:,idx])   #Must use this format with numpy array for stack of images. img_stack_array[:,:,idx]
            avg_down,probe_contour = img_proc.contour()  #Get contour returns (avg,points)

            #Assign a value to every image of the probe and later use these list of values to find the index corresponding to the probe tip.
            if np.isnan(avg_down):
                u = np.append(u,np.nan)

            else:
                #Use an image process function to determine the U shape (quality of the probe tip in focus)

                # Calculate Harris Corners
                num_corners, corners = img_proc.harris_corner(img_stack_array[:,:,idx])

                #for corner in range(len(corners)):
                #    cv.circle(img_stack_array[:,:,idx], (corners[corner][0],corners[corner][1]), 2, (255), 2)
                
                #cv.imwrite('C:/Users/Lab/Images/creep sequence/' + str(idx) + '.bmp', img_stack_array[:,:,idx])   #img_stack_array[idx] img_stack_array[:,:,idx]

                u = np.append(u,num_corners)

        print(u)

        #Find the index in the list with minimum harris corners
        min_idx = np.nanargmin(u)  #index of least number of corners excluding nan

        print(min_idx)
        print(self.image_stack_coord[min_idx])

        #Move slicescope to the location of the min_idx and the slicescope z coordinate
        self.slicescope_instance.moveAbsolute(self.slicescope_instance.x,self.slicescope_instance.y,self.image_stack_coord[min_idx])
