# Dr. Vincent Lee
# tkgui.py
# Description:
# Tkinter GUI classes for GUI frames

from tkinter import *
from PIL import Image, ImageTk

import time
import numpy as np
import cv2 as cv
import matplotlib.pyplot as plt

from image_window import Image_window
from image_process import Image_process
from camera import Camera
from threading import Thread

from skimage.restoration import denoise_tv_chambolle

from heka import Heka
from batch import Batch

class Window:

    def __init__(self, slicescope, condenser, cam, patchstar):

        self.slicescope_instance = slicescope
        self.condenser_instance = condenser
        self.pvcam_instance = cam
        self.patchstar_instance = patchstar

        self.window = Toplevel()

        self.window_label = Label(self.window)
        self.window_label.pack()

        self.open_flag = True

        thread = Thread(target = self.video_loop())
        thread.setDaemon(True)
        thread.start()

        self.window.wm_protocol("WM_DELETE_WINDOW", self.stop_video)

    #####----------FUNCTIONS----------#####

    def video_loop(self):

        cap = self.pvcam_instance.get_frame()
        img = Image.fromarray(cap)
        photo_img = ImageTk.PhotoImage(image = img)

        self.window_label.image = photo_img
        self.window_label.configure(image = photo_img)

        if (self.open_flag == True):
            self.window.after(10, self.video_loop)  #milliseconds

    def stop_video(self):
        self.open_flag = False
        self.window.destroy()

######################################################################################################################################################

class App(Tk):

    def __init__(self, slicescope, condenser, cam, patchstar):

        self.slicescope_instance = slicescope
        self.condenser_instance = condenser
        self.pvcam_instance = cam
        self.patchstar_instance = patchstar

        self.window = []

        self.k_centroid_list = []
        self.tip_in_focus_corners = []
        self.offset_x_pixels = []
        self.offset_y_pixels = []
        self.tip_angle = []

        self.target1 = np.zeros((3,), dtype = int)
        self.target2 = np.zeros((3,), dtype = int)


        self.creepZ = 0

        #HEKA batch comm setting
        self.batch_flag = []

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
        self.submenu_frame_label1 = Label(self.submenu_frame1, text = "Camera Control", font = ("Arial",15,"bold"), padx=18)
        self.submenu_frame_label1.pack()

        btn_video = Button(self.submenu_frame1, text = "VIDEO", font = btn_font, width = btn_width, height = btn_height, command = self.video, relief = RAISED, borderwidth = btn_border)
        btn_video.pack()

        self.label = Label(self.submenu_frame1, text = "Move Camera (um)", font = btn_font)
        self.label.pack()
        self.scale_txt = Entry(self.submenu_frame1, width = 15)
        self.scale_txt.pack()

        btn_set_scale = Button(self.submenu_frame1, text = "Set", command = lambda: self.get_step(self.scale_txt), font = btn_font)
        btn_set_scale.pack()

        btn_left = Button(self.submenu_frame1, text = "LEFT", font = btn_font, width = btn_width, height = btn_height, command = lambda: self.slicescope_instance.moveRelative(self.slicescope_instance.x,self.slicescope_instance.y,self.slicescope_instance.z,-int(self.entry_step),0,0), relief = RAISED, borderwidth = btn_border)
        btn_left.pack()
        btn_right = Button(self.submenu_frame1, text = "RIGHT", font = btn_font, width = btn_width, height = btn_height, command = lambda: self.slicescope_instance.moveRelative(self.slicescope_instance.x,self.slicescope_instance.y,self.slicescope_instance.z,int(self.entry_step),0,0), relief = RAISED, borderwidth = btn_border)
        btn_right.pack()
        btn_up = Button(self.submenu_frame1, text = "UP", font = btn_font, width = btn_width, height = btn_height, command = lambda: self.slicescope_instance.moveRelative(self.slicescope_instance.x,self.slicescope_instance.y,self.slicescope_instance.z,0,-int(self.entry_step),0), relief = RAISED, borderwidth = btn_border)
        btn_up.pack()
        btn_down = Button(self.submenu_frame1, text = "DOWN", font = btn_font, width = btn_width, height = btn_height, command = lambda: self.slicescope_instance.moveRelative(self.slicescope_instance.x,self.slicescope_instance.y,self.slicescope_instance.z,0,int(self.entry_step),0), relief = RAISED, borderwidth = btn_border)
        btn_down.pack()

        btn_zoom_in = Button(self.submenu_frame1, text = "IN", font = btn_font, width = btn_width, height = btn_height, command = lambda: self.slicescope_instance.moveRelative(self.slicescope_instance.x,self.slicescope_instance.y,self.slicescope_instance.z,0,0,-int(self.entry_step)), relief = RAISED, borderwidth = btn_border)
        btn_zoom_in.pack()
        btn_zoom_out = Button(self.submenu_frame1, text = "OUT", font = btn_font, width = btn_width, height = btn_height, command = lambda: self.slicescope_instance.moveRelative(self.slicescope_instance.x,self.slicescope_instance.y,self.slicescope_instance.z,0,0,int(self.entry_step)), relief = RAISED, borderwidth = btn_border)
        btn_zoom_out.pack()


        self.approach_label = Label(self.submenu_frame1, text = "Move Approach (um)", font = btn_font)
        self.approach_label.pack()
        self.approach_scale_txt = Entry(self.submenu_frame1, width = 15)
        self.approach_scale_txt.pack()

        btn_approach_set_scale = Button(self.submenu_frame1, text = "Set", command = lambda: self.get_approach_step(self.approach_scale_txt), font = btn_font)
        btn_approach_set_scale.pack()

        btn_approach = Button(self.submenu_frame1, text = "Approach", font = btn_font, width = btn_width, height = btn_height, command = lambda: self.approach(int(self.entry_approach_step)), relief = RAISED, borderwidth = btn_border)
        btn_approach.pack()

    ################################################################

        #self.submenu_frame2 = Frame(self.sidebar, bg = submenu_color, relief = RAISED, borderwidth = menu_border)
        #self.submenu_frame2.pack()
        #self.submenu_frame_label2 = Label(self.submenu_frame2, text = "CAL", font = ("Arial",15,"bold"), padx=73)
        #self.submenu_frame_label2.pack()

        #btn_slicescope_calibration = Button(self.submenu_frame2, text = "Slicescope CAL", font = btn_font, width = btn_width, height = btn_height, command = self.slicescope_calibration, relief = RAISED, borderwidth = btn_border)
        #btn_slicescope_calibration.pack()
        #btn_patchstar_calibration = Button(self.submenu_frame2, text = "Patchstar CAL", font = btn_font, width = btn_width, height = btn_height, command = self.patchstar_calibration, relief = RAISED, borderwidth = btn_border)
        #btn_patchstar_calibration.pack()

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

        #btn_low_mag_find_tip = Button(self.submenu_frame4, text = "Find tip", font = btn_font, width = btn_width, height = btn_height, command = lambda: self.low_mag_find_tip(int(self.entry_distance),int(self.entry_speed)), relief = RAISED, borderwidth = btn_border)
        #btn_low_mag_find_tip.pack()

        #btn_creep_a = Button(self.submenu_frame4, text = "Creep A", font = btn_font, width = btn_width, height = btn_height, command = lambda: self.patchstar_instance.creepA(int(self.entry_distance),int(self.entry_speed)), relief = RAISED, borderwidth = btn_border)
        #btn_creep_a.pack()

        #btn_track_probe_tip = Button(self.submenu_frame4, text = "Track probe tip", font = btn_font, width = btn_width, height = btn_height, command = lambda: self.track_probe_tip(int(self.entry_distance),int(self.entry_speed)), relief = RAISED, borderwidth = btn_border)
        #btn_track_probe_tip.pack()

        btn_img_stack_creep = Button(self.submenu_frame4, text = "Image Stack Creep", font = btn_font, width = btn_width, height = btn_height, command = lambda: self.img_stack_creep(int(self.entry_distance),int(self.entry_speed)), relief = RAISED, borderwidth = btn_border)
        btn_img_stack_creep.pack()


#####----------FUNCTIONS----------#####

    def get_step(self, scale_txt):
        self.entry_step = scale_txt.get()

        if int(self.entry_step) < 1_00:
            self.entry_step = 0
        else:
            self.entry_step = int(self.entry_step)

    def get_focus_step(self, focus_scale_txt):
        self.entry_focus_step = focus_scale_txt.get()

        if int(self.entry_focus_step) < 1_00:
            self.entry_focus_step = 0
        else:
            self.entry_focus_step = int(self.entry_focus_step)

    def get_approach_step(self, approach_scale_txt):
        self.entry_approach_step = int(approach_scale_txt.get())

    def get_distance(self, distance_scale_txt):
        self.entry_distance = int(distance_scale_txt.get())

    def get_speed(self, speed_scale_txt):
        self.entry_speed = speed_scale_txt.get()

        if int(self.entry_speed) < 1_00:
            self.entry_speed = 0
        else:
            self.entry_speed = int(self.entry_speed)


    def approach(self,step):

        #Move micromanipulator (patchstar) probe IN and OUT using APPROACH
        self.patchstar_instance.approachRelative(self.patchstar_instance.a,int(step))

        self.patchstar_instance.coordinates()

        print(f'Micromanipulator: X: {self.patchstar_instance.x}, Y: {self.patchstar_instance.y}, Z: {self.patchstar_instance.z}, A: {self.patchstar_instance.a}')

    def video(self):
        self.window = Window(self.slicescope_instance, self.condenser_instance, self.pvcam_instance, self.patchstar_instance)

    def get_snapshot(self, label):
        cap = self.pvcam_instance.get_frame()  #See PVCAM for exp. time in milliseconds
        img = Image.fromarray(cap)
        photo_img = ImageTk.PhotoImage(image = img)

        label.image = photo_img
        label.configure(image = photo_img)

        return cap

    def slicescope_calibration(self):

        #Save current coordinate as condenser origin
        self.condenser_instance.coordinates()
        self.condenser_instance.z_origin = self.condenser_instance.z
        
        #Move condenser out of the way
        self.condenser_instance.moveRelative(self.condenser_instance.z_origin,100000_00)

        #Run Slicescope calibration
        self.slicescope_instance.calibration()

        #Move condenser back to original position
        self.condenser_instance.moveAbsolute(self.condenser_instance.z_origin)

    def patchstar_calibration(self):
        self.patchstar_instance.calibration()

    def update_coordinates(self):

        #Update current position of slicescope and patchstar.
        self.slicescope_instance.coordinates()
        self.patchstar_instance.coordinates()

        #Update deltas w.r.t. maximum limits of slicescope and patchstar using current coordinates.
        self.slicescope_instance.deltas()
        self.patchstar_instance.deltas()

        print(f'Slicescope: X: {self.slicescope_instance.x}, Y: {self.slicescope_instance.y}, Z: {self.slicescope_instance.z}')
        print(f'Micromanipulator: X: {self.patchstar_instance.x}, Y: {self.patchstar_instance.y}, Z: {self.patchstar_instance.z}, A: {self.patchstar_instance.a}')


    def heka_iv_monitor(self):

        heka_proc = Heka()
        batch_proc = Batch()

        heka_proc.initialize()

        time.sleep(0.1)
        
        thread_batch = Thread(target = batch_proc.batch_comm())
        thread_batch.setDaemon(True)
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
        thread_batch.setDaemon(True)
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
        thread_batch.setDaemon(True)
        thread_batch.start()

        self.batch_flag = True
        
        time.sleep(0.1)

        heka_proc.Rpip_monitor(self.batch_flag)


    def stop_batch(self):

        batch_proc = Batch()

        batch_proc.batch_comm()
        self.batch_flag = False


    def low_mag_find_tip(self, distance, speed):

        img_proc = Image_process()

        #Initialization
        elapsed_loop = 0
        cnt = 0
        u = []
        img_stack = []

        #Move slicescope using Creep Z
        loop_time = np.ceil(abs(distance/speed)) # micrometers / micrometers/second
        self.slicescope_instance.creepZ(distance, abs(speed))

        while elapsed_loop < loop_time:
            tic = time.time()

            #update the video feed in window_label
            self.window.window_label.update()

            cap = self.get_snapshot(self.window.window_label)
            img_stack.append(cap)

            #Use contour to determine if any part of the probe is in focus
            img_proc.load_frame(cap)
            avg_down,probe_contour = img_proc.contour()  #Get contour returns (avg,points)

            #Assign a value to every image of the probe and later use these list of values to find the index corresponding to the probe tip.
            if np.isnan(avg_down):
                u = np.append(u,np.nan)

            else:
                #Use an image process function to determine the U shape (quality of the probe tip in focus)

                # Calculate Harris Corners
                num_corners, corners = img_proc.harris_corner(cap)

                u = np.append(u,num_corners)

            toc = time.time()
            elapsed_loop += toc-tic

            print(f"iteration = {cnt}")
            cnt = cnt + 1

        print(f'Loop Time = {loop_time:.2f} seconds')
        print(f'Time elapsed = {elapsed_loop:.2f} seconds')

        #Convert the list of images into a numpy array. Dimensions are (Num frames, Height, Width)
        img_stack_array = np.array(img_stack)

        #Move the index for the number of images in array to the end. Dimensions are (Height, Width, Num frames)
        img_stack_array = np.moveaxis(img_stack_array, 0, -1)

        #Save or show images
        for frame in range(img_stack_array.shape[2]):
            cv.imwrite('C:/Users/Lab/Images/' + str(frame) + '.bmp', img_stack_array[:,:,frame])
            #cv.imshow('frame', img_stack_array[:,:,frame])   #Each array is displayed using all pixels in [height,width,and iterated through one frame at a time].
            #cv.waitKey(200)

        #Calculate the average distance traversed per image
        average_dist = abs(distance)/img_stack_array.shape[2]

        #Find the index in the list with minimum harris corners
        min_idx = np.nanargmin(u)  #index of least number of corners excluding nan

        #Calculate how far away the min index is from the last index
        idx_from_end = img_stack_array.shape[2] - min_idx

        #Estimate the distance needed to move to the min index
        idx_dist = int(average_dist*idx_from_end)

        #Move slicescope up by idx_dist (should be the location of the min_idx)
        self.slicescope_instance.creepZ(idx_dist, idx_dist)

    def img_stack_creep(self, distance, speed):

        img_proc = Image_process()

        hc = []
        score = []

        seconds = 2

        numframes = 200

        #self.pvcam_instance.size[0] = height = 1024
        #self.pvcam_instance.size[1] = width = 1367
        stack = np.empty((numframes, self.pvcam_instance.size[0],self.pvcam_instance.size[1]))

        creep_time = abs(distance/speed)

        data_frames = np.round(((creep_time+seconds)/0.04)).astype(int)

        if data_frames > numframes:
            data_frames = numframes
        else:
            data_frames = data_frames

        #Setup non-circular buffer acquisition
        self.pvcam_instance.cam.start_seq(exp_time=10, num_frames=numframes)

        time.sleep(seconds)

        self.slicescope_instance.creepZ(distance, speed)

        tic = time.time()
        for idx in range(data_frames):
            #poll_frame takes ~0.03s per frame
            frame, fps, frame_count = self.pvcam_instance.cam.poll_frame()   #poll frame data from photo sequence buffer
            max_frame = np.max(frame['pixel_data'])
            stack[idx] = np.uint8(frame['pixel_data']/max_frame * 255)       
            image = np.uint8(frame['pixel_data']/max_frame * 255) 

            laplace_var = np.round(cv.Laplacian(stack[idx], cv.CV_64F).var())

            score.append(laplace_var)

            #Draw Harris Corners
            num_corners, corners = img_proc.harris_corner(image)

            for corner in range(len(corners)):
                cv.circle(image, (corners[corner][0],corners[corner][1]), 2, (255), 2)

            hc.append(num_corners)

            #cv.imwrite('C:/Users/Lab/Images/creep sequence/' + str(idx) + '.bmp', image)
            #cv.imwrite('C:/Users/Lab/Images/creep sequence/' + str(idx) + '_' + str(laplace_var) + '.bmp', stack[idx])


        toc = time.time()
        elapsed_per_frame = (toc-tic)/numframes

        print(f'Loop time per frame = {elapsed_per_frame:.3f} seconds')

        self.pvcam_instance.cam.finish()

        #Test out step function response normalized between -1 and 1.
        #This function gives you the approximate center, start and end points of the movement.
        x = np.array(score).astype('float')
        lower_bound = -1
        upper_bound = 1
        x = ((upper_bound - lower_bound)*(x - min(x)) / (max(x) - min(x))) + lower_bound

        x = x - np.average(x)

        #if x[0] < 0:
        #    step = np.hstack((np.ones(len(x)), -1*np.ones(len(x))))  #step down function

        #else:
        #    step = np.hstack((-1*np.ones(len(x)), np.ones(len(x))))  #step up function

        #x_step = np.convolve(x,step,mode='valid')
        #step_idx = np.argmax(x_step)

        #plt.plot(x)
        #plt.plot(x_step/10)
        #plt.plot((step_idx,step_idx), (x_step[step_idx]/10, 0), 'r')

        x_denoise = denoise_tv_chambolle(x, weight=1)

        factor = 0.85
        lower_thresh = factor*np.min(x_denoise)
        upper_thresh = factor*np.max(x_denoise)

        if x_denoise[0] < 0:
            x_denoise_min_idx = np.argmax(x_denoise>lower_thresh)
            x_denoise_max_idx = np.argmax(x_denoise>upper_thresh)
        else:
            x_denoise_min_idx = np.argmax(x_denoise<upper_thresh)
            x_denoise_max_idx = np.argmax(x_denoise<lower_thresh)

        f1 = plt.figure()
        f1ax1 = f1.add_subplot(111)
        f1ax1.plot(x)
        f1ax1.plot(x_denoise)
        f1ax1.plot((x_denoise_min_idx,x_denoise_min_idx), (x_denoise[x_denoise_min_idx], 0), 'r')
        f1ax1.plot((x_denoise_max_idx,x_denoise_max_idx), (x_denoise[x_denoise_max_idx], 0), 'b')


        f2 = plt.figure()
        f2ax1 = f2.add_subplot(111)
        f2ax1.plot(hc)

        plt.show()