# Dr. Vincent Lee
# tkgui.py
# Description:
# Tkinter GUI classes for GUI frames

from tkinter import *
from PIL import Image, ImageTk

import time
import numpy as np
import matplotlib.pyplot as plt
import cv2 as cv

from image_window import Image_window
from image_process import Image_process
from camera import Camera
from threading import Thread

from skimage import io, img_as_float
from skimage.restoration import denoise_nl_means, estimate_sigma, denoise_tv_chambolle

from scipy.signal import find_peaks

import win32com.client as win32
import win32gui


class Batch:

    def __init__(self):

        thread_batch = Thread(target = self.batch_comm())
        thread_batch.setDaemon(True)
        thread_batch.start()

    #####----------FUNCTIONS----------#####

    def windowEnumerationHandler(self, hwnd, top_windows):
        top_windows.append((hwnd, win32gui.GetWindowText(hwnd)))

    def batch_comm(self):

        #title = "PATCHMASTER NEXT"  # cycle all windows with this title

        top_windows = []  # all open windows
        win32gui.EnumWindows(self.windowEnumerationHandler, top_windows)
        #print(top_windows)

        winlst = []  # windows to cycle through
        for i in top_windows:  # all open windows
            #if i[1] == title:
            #    winlst.append(i)
            if "Configuration" in i[1]:   # Check if "Configuration" is in window title to isolate Configuration window.
                winlst.append(i)

        for w in winlst:  # each window with selected title
            shell = win32.Dispatch("WScript.Shell")  # set focus on desktop
            shell.SendKeys('%')  # Alt key
            win32gui.SetForegroundWindow(w[0]) # bring to front, activate
            time.sleep(0.1)

            # WinShell presses 'b' for batch communications shortcut in Patchmaster Next.
            # batch communication b shortcut can remain local, not global.
            # This remains a default setting in PatchMaster Next.
            # b can be pressed again to close batch communication.
            shell.SendKeys('b')  # b key
            time.sleep(0.1)

class Window:

    def __init__(self, slicescope, condenser, patchstar, cam):

        self.slicescope_instance = slicescope
        self.condenser_instance = condenser
        self.patchstar_instance = patchstar
        self.pvcam_instance = cam

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

        self.window_label.photo_image = photo_img
        self.window_label.configure(image = photo_img)

        if (self.open_flag == True):
            self.window.after(10, self.video_loop)

    def stop_video(self):
        self.open_flag = False
        self.window.destroy()

######################################################################################################################################################

class App(Tk):

    def __init__(self, slicescope, condenser, patchstar, cam):

        self.slicescope_instance = slicescope
        self.condenser_instance = condenser
        self.patchstar_instance = patchstar
        self.pvcam_instance = cam

        self.window = []

        self.k_centroid_list = []
        self.tip_in_focus_corners = []
        self.offset_x_pixels = []
        self.offset_y_pixels = []
        self.tip_angle = []

        self.target1 = np.zeros((3,), dtype = int)
        self.target2 = np.zeros((3,), dtype = int)

        #HEKA batch comm setting
        self.batch_flag = []

        #main_frame_color = "white"
        sidebar_color = "gray"
        submenu_color = "blue"

        #gui_width = 1800
        #gui_height = 1000
        #screen_dimensions = str(gui_width) + 'x' + str(gui_height)

        Tk.__init__(self)
        self.title("Bioptix Ltd")
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

        #Main Frame
        #self.main_frame_width = 1000
        #self.main_frame_height = 1000
        #self.main_frame_border = 0
        #self.main_frame_thickness = 0
        #self.main_frame = Frame(self, bg = main_frame_color, width = self.main_frame_width, height = self.main_frame_height, relief = RAISED, borderwidth = self.main_frame_border, highlightthickness = self.main_frame_thickness)
        #self.main_frame.pack(side = LEFT, fill = BOTH, expand = TRUE)
        #self.main_frame_label = Label(self.main_frame, text = "Main Frame", font = ("Arial",15,"bold"))
        #self.main_frame_label.pack_propagate(FALSE)
       
        #self.main_frame = Frame(self, bg = main_frame_color)
        #self.main_frame.pack()
        #self.main_frame_label = Label(self.main_frame)
        #self.main_frame_label.pack()

        #Sidebar
        #self.sidebar = Frame(self, bg = sidebar_color, width = 200, height = gui_height, relief = RAISED, borderwidth = menu_border)
        self.sidebar = Frame(self, bg = sidebar_color, relief = RAISED, borderwidth = menu_border)
        self.sidebar.pack(side = LEFT, fill = BOTH, expand = FALSE)

        #Logo on top in sidebar
        #self.logo = Frame(self.sidebar, bg = sidebar_color, width = 100, height = 100, relief = RAISED, borderwidth = menu_border)
        self.logo = Frame(self.sidebar, bg = sidebar_color, relief = RAISED, borderwidth = menu_border)
        self.logo.pack()
        self.logo_label = Label(self.logo, text = "Bioptix Ltd", font = ("Arial",15,"bold"), padx=30)
        self.logo_label.pack()

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

        self.submenu_frame1 = Frame(self.sidebar, bg = submenu_color, relief = RAISED, borderwidth = menu_border)
        self.submenu_frame1.pack()
        self.submenu_frame_label1 = Label(self.submenu_frame1, text = "Camera Control", font = ("Arial",15,"bold"), padx=7)
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

        self.submenu_frame2 = Frame(self.sidebar, bg = submenu_color, relief = RAISED, borderwidth = menu_border)
        self.submenu_frame2.pack()
        self.submenu_frame_label2 = Label(self.submenu_frame2, text = "Low Mag", font = ("Arial",15,"bold"), padx=38)
        self.submenu_frame_label2.pack()

        btn_slicescope_calibration = Button(self.submenu_frame2, text = "Slicescope CAL", font = btn_font, width = btn_width, height = btn_height, command = self.slicescope_calibration, relief = RAISED, borderwidth = btn_border)
        btn_slicescope_calibration.pack()
        btn_patchstar_calibration = Button(self.submenu_frame2, text = "Patchstar CAL", font = btn_font, width = btn_width, height = btn_height, command = self.patchstar_calibration, relief = RAISED, borderwidth = btn_border)
        btn_patchstar_calibration.pack()

        btn_autofocus = Button(self.submenu_frame2, text = "AUTOfocus", font = btn_font, width = btn_width, height = btn_height, command = self.autofocus, relief = RAISED, borderwidth = btn_border)
        btn_autofocus.pack()

        #btn_harris_corner = Button(self.submenu_frame2, text = "Harris Corner", font = btn_font, width = btn_width, height = btn_height, command = self.harris_corner, relief = RAISED, borderwidth = btn_border)
        #btn_harris_corner.pack()

        #btn_houghline = Button(self.submenu_frame2, text = "Houghline", font = btn_font, width = btn_width, height = btn_height, command = self.houghline, relief = RAISED, borderwidth = btn_border)
        #btn_houghline.pack()

        #self.focus_label = Label(self.submenu_frame2, text = "Move Focus (um)", font = btn_font)
        #self.focus_label.pack()
        #self.focus_scale_txt = Entry(self.submenu_frame2, width = 15)
        #self.focus_scale_txt.pack()

        #btn_focus_set_scale = Button(self.submenu_frame2, text = "Set", command = lambda: self.get_focus_step(self.focus_scale_txt), font = btn_font)
        #btn_focus_set_scale.pack()

        #btn_focus_down = Button(self.submenu_frame2, text = "Focus DOWN", font = btn_font, width = btn_width, height = btn_height, command = lambda: self.focus_down(int(self.entry_focus_step)), relief = RAISED, borderwidth = btn_border)
        #btn_focus_down.pack()

        #btn_focus_up = Button(self.submenu_frame2, text = "Focus UP", font = btn_font, width = btn_width, height = btn_height, command = lambda: self.focus_up(int(self.entry_focus_step)), relief = RAISED, borderwidth = btn_border)
        #btn_focus_up.pack()

        ##### Use this for low magnification #####
        #btn_find_tip_corners = Button(self.submenu_frame2, text = "Find TIP", font = btn_font, width = btn_width, height = btn_height, command = self.find_tip_corners, relief = RAISED, borderwidth = btn_border)
        #btn_find_tip_corners.pack()
        #####

    ################################################################

        self.submenu_frame3 = Frame(self.sidebar, bg = submenu_color, relief = RAISED, borderwidth = menu_border)
        self.submenu_frame3.pack()
        self.submenu_frame_label3 = Label(self.submenu_frame3, text = "High Mag", font = ("Arial",15,"bold"), padx=38)
        self.submenu_frame_label3.pack()

        #btn_high_mag_roi = Button(self.submenu_frame3, text = "HIGH MAG ROI", font = btn_font, width = btn_width, height = btn_height, command = self.high_mag_roi, relief = RAISED, borderwidth = btn_border)
        #btn_high_mag_roi.pack()

        #btn_non_local_means_cv = Button(self.submenu_frame3, text = "NLM - CV", font = btn_font, width = btn_width, height = btn_height, command = self.non_local_means_CV, relief = RAISED, borderwidth = btn_border)
        #btn_non_local_means_cv.pack()

        #btn_non_local_means_sk = Button(self.submenu_frame3, text = "NLM - SK", font = btn_font, width = btn_width, height = btn_height, command = self.non_local_means_SK, relief = RAISED, borderwidth = btn_border)
        #btn_non_local_means_sk.pack()

        btn_high_mag_into_water = Button(self.submenu_frame3, text = "Move into water", font = btn_font, width = btn_width, height = btn_height, command = self.high_mag_into_water, relief = RAISED, borderwidth = btn_border)
        btn_high_mag_into_water.pack()

        btn_high_mag_on_probe = Button(self.submenu_frame3, text = "Focus on probe", font = btn_font, width = btn_width, height = btn_height, command = self.high_mag_on_probe, relief = RAISED, borderwidth = btn_border)
        btn_high_mag_on_probe.pack()

        btn_high_mag_on_tip = Button(self.submenu_frame3, text = "Focus on tip", font = btn_font, width = btn_width, height = btn_height, command = self.high_mag_on_tip, relief = RAISED, borderwidth = btn_border)
        btn_high_mag_on_tip.pack()

        btn_minor_high_mag_autofocus = Button(self.submenu_frame3, text = "Minor Autofocus", font = btn_font, width = btn_width, height = btn_height, command = self.minor_high_mag_autofocus, relief = RAISED, borderwidth = btn_border)
        btn_minor_high_mag_autofocus.pack()

        btn_high_mag_autofocus = Button(self.submenu_frame3, text = "Major Autofocus", font = btn_font, width = btn_width, height = btn_height, command = self.high_mag_autofocus, relief = RAISED, borderwidth = btn_border)
        btn_high_mag_autofocus.pack()

    ################################################################

        self.submenu_frame4 = Frame(self.sidebar, bg = submenu_color, relief = RAISED, borderwidth = menu_border)
        self.submenu_frame4.pack()
        self.submenu_frame_label4 = Label(self.submenu_frame4, text = "HEKA", font = ("Arial",15,"bold"), padx=38)
        self.submenu_frame_label4.pack()

        #User opens Patchmaster Next and loads their configuration. Make sure b is the GLOBAL shortcut in Patchmaster Next

        btn_batch_iv_monitor = Button(self.submenu_frame4, text = "Batch I-V", font = btn_font, width = btn_width, height = btn_height, command = self.heka_iv_monitor, relief = RAISED, borderwidth = btn_border)
        btn_batch_iv_monitor.pack()

        btn_batch_loop_monitor = Button(self.submenu_frame4, text = "Batch Loop", font = btn_font, width = btn_width, height = btn_height, command = self.heka_loop_monitor, relief = RAISED, borderwidth = btn_border)
        btn_batch_loop_monitor.pack()

        btn_batch_Rpip_monitor = Button(self.submenu_frame4, text = "Batch Rpip", font = btn_font, width = btn_width, height = btn_height, command = self.heka_Rpip_monitor, relief = RAISED, borderwidth = btn_border)
        btn_batch_Rpip_monitor.pack()

        btn_stop_batch = Button(self.submenu_frame4, text = "Stop Batch", font = btn_font, width = btn_width, height = btn_height, command = self.stop_batch, relief = RAISED, borderwidth = btn_border)
        btn_stop_batch.pack()

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
        self.entry_approach_step = approach_scale_txt.get()
        self.entry_approach_step = int(self.entry_approach_step)

    def box_crosshairs(self):

        cam_control = Camera()
        cap = self.pvcam_instance.get_frame()
        camera_frame = cam_control.box(cap)
        img = Image.fromarray(camera_frame)
        photo_img = ImageTk.PhotoImage(image = img)

        self.window_label.photo_image = photo_img
        self.window_label.configure(image = photo_img)

        if (self.open_flag == True):
            self.window_label.after(10, self.box_crosshairs)

    def approach(self,step):

        #Move micromanipulator (patchstar) probe IN and OUT using APPROACH
        self.patchstar_instance.approachRelative(self.patchstar_instance.a,int(step))

    def video(self):
        self.window = Window(self.slicescope_instance, self.condenser_instance, self.patchstar_instance, self.pvcam_instance)

    def get_snapshot(self, label):
        cap = self.pvcam_instance.get_frame()
        img = Image.fromarray(cap)
        photo_img = ImageTk.PhotoImage(image = img)

        label.photo_image = photo_img
        label.configure(image = photo_img)

        return cap

    def calibration(self):
        self.slicescope_instance.calibration()
        self.patchstar_instance.calibration()

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

    def focus_down(self,step):
        img_proc = Image_process()

        self.update_coordinates()

        if step < 1_00:
            step = 1_00

        #Move slicescope down until probe is out of focus
        cnt = 0
        while True:
            cnt = cnt + 1
            self.slicescope_instance.moveRelative(self.slicescope_instance.x,self.slicescope_instance.y,self.slicescope_instance.z,0,0,-int(step))

            #update the video feed in window_label
            self.window.window_label.update()

            cap = self.get_snapshot(self.window.window_label)
            img_proc.load_frame(cap) 
            avg_down,probe_contour = img_proc.contour()  #Get contour (avg,points) 
            print(f"avg_down = {avg_down}")

            if np.isnan(avg_down):
                break

            print(f"iteration = {cnt}")

    def plot_graph(self, ax_handle, x, y):
        ax_handle.plot(x, y, 'k-')

    def focus_up(self, step):
        img_proc = Image_process()

        self.update_coordinates()

        #Setup figure properties for matplotlib
        fig = plt.figure()
        ax = fig.add_subplot(1, 1, 1)
        ax.set(title = 'Convergence on the probe tip', xlabel = 'Window number', ylabel = 'Moving average (number of corners)')
        #ax.set(title = 'Convergence on the probe tip', xlabel = 'Count index', ylabel = 'Number of corners')
        plt.show(block=False)

        if step < 2_00:
            step = 2_00

        #Move slicescope up until probe is in focus
        #Calculate rolling average to find the minimum number of corners. This is when the probe is in focus.
        xs = []
        ys = []
        moving_averages = []
        focus_height = []
        focus_corners = []

        thread = Thread(target = self.plot_graph(ax, xs, moving_averages))
        #thread = Thread(target = self.plot_graph(ax, xs, ys))
        thread.setDaemon(True)
        thread.start()

        cnt = 0
        while True:
            cnt = cnt + 1
            self.slicescope_instance.moveRelative(self.slicescope_instance.x,self.slicescope_instance.y,self.slicescope_instance.z,0,0,int(step))
            
            #update the video feed in window_label
            self.window.window_label.update()

            cap = self.get_snapshot(self.window.window_label)
            img_proc.load_frame(cap) 
            avg_up, avg_up_contour = img_proc.contour()
            print(f"avg_up = {avg_up}")

            num_corners, corners = img_proc.harris_corner(cap)
            print(f"Number of corners detected = {num_corners}")

            xs.append(cnt)
            ys.append(num_corners)
            focus_corners.append(corners)
            focus_height.append(self.slicescope_instance.z)

            window_size = 5

            if len(ys) < window_size:
                moving_averages.append(np.nan)
            else:
                #Calculate the average of current window
                window_average = round(np.sum(ys[-window_size:])/window_size)
                moving_averages.append(window_average)

            #Plot using matplotlib
            #These codes work to constantly update the plot in real time, but is slow.
            #Moving averages real time graph
            self.plot_graph(ax, xs, moving_averages)
            plt.pause(0.001)

            #Harris corners real time graph
            #self.plot_graph(ax,xs,ys)
            #plt.pause(0.001)

            if cnt == 100:

                #Find the first minimum number of corners and use index to save the points and move slicescope to appropriate focus height
                min_convergence_index = moving_averages.index(np.nanmin(moving_averages))
                #min_convergence_index = ys.index(np.nanmin(ys))
                tip_in_focus_height = focus_height[min_convergence_index]
                tip_in_focus_corners = focus_corners[min_convergence_index]
                self.slicescope_instance.moveAbsolute(self.slicescope_instance.x,self.slicescope_instance.y,int(tip_in_focus_height))

                print(f"Tip in Focus - Slicescope Z = {tip_in_focus_height}")
                print(f"Tip in Focus - Harris Corners = {tip_in_focus_corners}")

                #k_centroid_list,k_sse_list = img_proc.k_means(corners) as reference
                k_centroid_list,k_sse_list = img_proc.k_means(tip_in_focus_corners)
                k_centroid_list = np.reshape(k_centroid_list,[-1,2])
                print("k centroid list")
                print(k_centroid_list)

                #update the video feed in window_label
                self.window.window_label.update()

                self.k_centroid_list = k_centroid_list
                self.tip_in_focus_corners = tip_in_focus_corners

                break

            print(f"iteration = {cnt}")

    def find_probe_direction_cropped(self):
        img_proc = Image_process()        

        #Find probe direction and tip
        img_tip = self.get_snapshot(self.window.window_label)
        cropped_img_tip, tip_boundary, tip_points, x_shift, y_shift = img_proc.detect_probe(img_tip)

        (x1,y1),(x2,y2),tip_angle = img_proc.detect_probe_direction(cropped_img_tip, tip_boundary, tip_points)

        print(f'(x1,y1) = ({x1},{y1})')
        print(f'(x2,y2) = ({x2},{y2})')

        print(f'(x_shift,y_shift) = ({x_shift},{y_shift})')

        #update the video feed in window_label
        self.window.window_label.update()

        if np.isnan(x2) or np.isnan(y2):
            pass

        else:

            self.tip_angle = tip_angle

            #Find Quadrant and offset from center
            offset_x_pixels, offset_y_pixels = img_proc.quadrant(cropped_img_tip, x2, y2)
            self.offset_x_pixels = offset_x_pixels + x_shift
            self.offset_y_pixels = offset_y_pixels + y_shift

            #add in the shift from cropped image back to full size camera image
            x1 = x1 + x_shift
            y1 = y1 + y_shift
            x2 = x2 + x_shift
            y2 = y2 + y_shift

            print(f'new (x1,y1) = ({x1},{y1})')
            print(f'new (x2,y2) = ({x2},{y2})')

            self.patchstar_instance.tip_coord_x = x2
            self.patchstar_instance.tip_coord_y = y2

            return x1,y1,x2,y2

    def find_tip_corners(self,x1,y1,x2,y2):
        img_proc = Image_process()

        #(x2,y2) is your estimated tip coordinate
        if (np.isnan(x2)) or (np.isnan(y2)):
            pass

        else:

            #Create new window for estimated tip coordinate
            img_wnd = Image_window(size = self.pvcam_instance.size)
            img_wnd.frame = self.pvcam_instance.get_frame()
            img_wnd.clear_overlay()

            height = img_wnd.frame.shape[0]
            width = img_wnd.frame.shape[1]

            #Find tip
            corner_distance = []

            # Loop over each data point
            for i in range( len(self.tip_in_focus_corners) ):
                x = self.tip_in_focus_corners[i]

                # Calculate distance from each k centroid to estimated probe tip (arrow tip)
                dist = img_proc.compute_L2_distance(x, [x2,y2])
                corner_distance.append(dist)

            #print(corner_distance)

            closest_corner_index =  corner_distance.index(min(corner_distance))

            probe_tip = self.tip_in_focus_corners[closest_corner_index]
            #print("probe tip calculation using min L2 distance")
            #print(probe_tip)
            probe_tip_x = int(probe_tip[0])
            probe_tip_y = int(probe_tip[1])

            #offset_x_pixels, offset_y_pixels = img_proc.quadrant(img_tip, x2, y2)
            #cv.circle(img_tip, (int(probe_tip[0]),int(probe_tip[1])), radius = 10, color = (255), thickness = 10)

            # Draw corners
            #for corner in range( len(self.tip_in_focus_corners) ):
            #    cv.circle(img_wnd.frame, (int(self.tip_in_focus_corners[corner][0]),int(self.tip_in_focus_corners[corner][1])), radius = 2, color = (255), thickness = 2)

            #cv.imshow('Probe tip corners', img_wnd.frame)
            #cv.waitKey(3)

            #Draw probe tip coordinate on window

            dot = cv.circle(img_wnd.frame,(probe_tip_x,probe_tip_y),radius=10,color=(255),thickness=10)
            dot_copy = dot.copy()
            dot_arrow = cv.arrowedLine(dot_copy,(x1,y1),(x2,y2),color=(255),thickness=10)

            font = cv.FONT_HERSHEY_SIMPLEX
            fontScale = 1
            color = (0)
            thickness = 2

            text_width_loc = int(0.3*width)

            org1 = (text_width_loc,100)
            text1 = 'Probe tip coordinate: (' + str(probe_tip_x)+ ', ' + str(probe_tip_y) + ')'
            dot = cv.putText(dot, text1, org1, font, fontScale, color, thickness, cv.LINE_AA, False)
            dot_arrow = cv.putText(dot_arrow, text1, org1, font, fontScale, color, thickness, cv.LINE_AA, False)

            format_angle = format(self.tip_angle, '0.2f')
            org2 = (text_width_loc,150)
            text2 = 'Probe angle w.r.t. Y-axis: ' + str(format_angle) + ' deg'
            dot = cv.putText(dot, text2, org2, font, fontScale, color, thickness, cv.LINE_AA, False)
            dot_arrow = cv.putText(dot_arrow, text2, org2, font, fontScale, color, thickness, cv.LINE_AA, False)

            #Show probe tip in separate window
            cv.imshow('Probe tip', dot)
            cv.imshow('Probe direction', dot_arrow)
            cv.waitKey(3)

            #Save image to file
            cv.imwrite('./probe_tip.png', dot)
            cv.imwrite('./probe_direction.png', dot_arrow)
            #cv.imwrite('./probe_tip_corners.png', img_wnd.frame)

    def find_tip_centroids(self,x1,y1,x2,y2):
        img_proc = Image_process()

        #(x2,y2) is your estimated tip coordinate
        if (np.isnan(x2)) or (np.isnan(y2)):
            pass

        else:

            #Create new window for estimated tip coordinate
            img_wnd = Image_window(size = self.pvcam_instance.size)
            img_wnd.frame = self.pvcam_instance.get_frame()
            img_wnd.clear_overlay()

            height = img_wnd.frame.shape[0]
            width = img_wnd.frame.shape[1]

            #Find tip
            centroid_distance = []

            # Loop over each data point
            for i in range( len(self.k_centroid_list) ):
                x = self.k_centroid_list[i]

                # Calculate distance from each k centroid to estimated probe tip (arrow tip)
                dist = img_proc.compute_L2_distance(x, [x2,y2])
                centroid_distance.append(dist)

            #print(corner_distance)

            closest_centroid_index =  centroid_distance.index(min(centroid_distance))

            probe_tip = self.k_centroid_list[closest_centroid_index]
            #print("probe tip calculation using min L2 distance")
            #print(probe_tip)
            probe_tip_x = int(probe_tip[0])
            probe_tip_y = int(probe_tip[1])

            #offset_x_pixels, offset_y_pixels = img_proc.quadrant(img_tip, x2, y2)
            #cv.circle(img_tip, (int(probe_tip[0]),int(probe_tip[1])), radius = 10, color = (255), thickness = 10)

            # Draw centroids
            #for coord in range( len(self.k_centroid_list) ):        
            #    cv.circle(img_wnd.frame, (int(self.k_centroid_list[coord][0]),int(self.k_centroid_list[coord][1])), radius = 2, color = (255), thickness = 2)
            #    cv.circle(img_wnd.frame, (int(self.k_centroid_list[coord][0]-x_shift),int(self.k_centroid_list[coord][1]-y_shift)), radius = 2, color = (255), thickness = 2)

            #cv.imshow('Probe tip centroids', img_wnd.frame)
            #cv.waitKey(3)

            #Draw probe tip coordinate on window

            dot = cv.circle(img_wnd.frame,(probe_tip_x,probe_tip_y),radius=10,color=(255),thickness=10)
            dot_copy = dot.copy()
            dot_arrow = cv.arrowedLine(dot_copy,(x1,y1),(x2,y2),color=(255),thickness=10)

            font = cv.FONT_HERSHEY_SIMPLEX
            fontScale = 1
            color = (0)
            thickness = 2

            text_width_loc = int(0.3*width)

            org1 = (text_width_loc,100)
            text1 = 'Probe tip coordinate: (' + str(probe_tip_x)+ ', ' + str(probe_tip_y) + ')'
            dot = cv.putText(dot, text1, org1, font, fontScale, color, thickness, cv.LINE_AA, False)
            dot_arrow = cv.putText(dot_arrow, text1, org1, font, fontScale, color, thickness, cv.LINE_AA, False)

            format_angle = format(self.tip_angle, '0.2f')
            org2 = (text_width_loc,150)
            text2 = 'Probe angle w.r.t. Y-axis: ' + str(format_angle) + ' deg'
            dot = cv.putText(dot, text2, org2, font, fontScale, color, thickness, cv.LINE_AA, False)
            dot_arrow = cv.putText(dot_arrow, text2, org2, font, fontScale, color, thickness, cv.LINE_AA, False)

            #Show probe tip in separate window
            cv.imshow('Probe tip', dot)
            cv.imshow('Probe direction', dot_arrow)
            cv.waitKey(3)

            #Save image to file
            cv.imwrite('./probe_tip.png', dot)
            cv.imwrite('./probe_direction.png', dot_arrow)
            #cv.imwrite('./probe_tip_centroids.png', img_wnd.frame)

    def autofocus(self):

        img_proc = Image_process()

        while True:

            cap = self.get_snapshot(self.window.window_label)
            time.sleep(0.1)

            #update the video feed in window_label
            self.window.window_label.update()

            #Check if the probe is in focus
            img_proc.load_frame(cap)   #Add the cam frame to the most recent img_list (img_list[-1]). Pass into image_process class for analysis
            check_focus,check_focus_contour = img_proc.contour()  #First contour (avg,points)
            print(f"Is the probe in focus? = {check_focus}")

            #User can either move probe into focus or more slicescope into focus. Once there is a focus on the probe, begin the loop.

            if not np.isnan(check_focus):

                self.update_coordinates()

                #Proceed to move down and up to autofocus on tip. 
                self.focus_down(10_00)

                self.focus_up(2_00)

                #Do not need to have probes move up and down together to get direction of probe.
                #Move slicescope and probe up together to remove background interference. Slicescope first, then micromanipulator.
                common_z_step = min(self.slicescope_instance.z_delta,self.patchstar_instance.z_delta)  #Low magnification reference delta that slicescope and patchstar need to ensure synchronization.
                #self.slicescope_instance.moveRelative(self.slicescope_instance.x,self.slicescope_instance.y,self.slicescope_instance.z,0,0,common_z_step)
                #self.patchstar_instance.moveRelative(self.patchstar_instance.x,self.patchstar_instance.y,self.patchstar_instance.z,0,0,common_z_step)

                #update the video feed in window_label
                self.window.window_label.update()

                #Find probe direction and tip within cropped window. This removes the Condenser ring shown on the main image.
                #Patchstar tip_coord_x and tip_coord_y are saved here with the x_shift and y_shift included
                x1,y1,x2,y2 = self.find_probe_direction_cropped()

                #Find the probe tip using saved Harris corners or K-means centroids list
                self.find_tip_corners(x1,y1,x2,y2)
                #self.find_tip_centroids(x1,y1,x2,y2)

                break

            else:
                print('Probe is OUT OF FOCUS')

        #Do not need to have probes move up and down together to get direction of probe.
        #Move slicescope and probe together back to starting point where probe tip was found. Micromanipulator first, then slicescope.
        #self.patchstar_instance.moveRelative(self.patchstar_instance.x,self.patchstar_instance.y,self.patchstar_instance.z,0,0,-common_z_step)
        #self.slicescope_instance.moveRelative(self.slicescope_instance.x,self.slicescope_instance.y,self.slicescope_instance.z,0,0,-common_z_step)

        #update the video feed in window_label
        #self.window.window_label.update()

#________________________________________________________________________________

        #Save the coordinates for slicescope and patchstar for later.
        #If user wants to take control of the scope and patchstar now and 
        #then come back to the automation later, these coordinates will help refocus the probe tip. 
        #Assuming patchstar is fixed to table. Only use the controllers to move the x,y,z,a. This will maintain all values relative to each other. 
        self.slicescope_instance.x_tip = self.slicescope_instance.x
        self.slicescope_instance.y_tip = self.slicescope_instance.y
        self.slicescope_instance.z_tip = self.slicescope_instance.z
        self.slicescope_instance.z_tip_dist = common_z_step
        self.patchstar_instance.x_tip = self.patchstar_instance.x
        self.patchstar_instance.y_tip = self.patchstar_instance.y
        self.patchstar_instance.z_tip = self.patchstar_instance.z
        self.patchstar_instance.z_tip_dist = common_z_step
        self.patchstar_instance.a_tip = self.patchstar_instance.a
        self.patchstar_instance.tip_angle = self.tip_angle   #float

        self.patchstar_instance.angle()
        self.patchstar_instance.a_angle = self.patchstar_instance.probe_angle

    def harris_corner(self, blockSize=2, kSize=3, k=0.04):

        img_wnd = Image_window(size = self.pvcam_instance.size)

        img_wnd.set_live_thread()
        img_wnd.thread.setDaemon(True)
        img_wnd.thread.start()

        # Wait for image window ready
        img_wnd.wait_window_ready()
        img_wnd.set_mouse_response()

        img_proc = Image_process()

        while True:
            # Generate random pixel map at a rate
            img_wnd.frame = self.pvcam_instance.get_frame()

            time.sleep(0.1)

            # If clicked a new coordinate, draw circle, and update coordinate
            if not img_wnd.click_coord == img_wnd.previous_click_coord:
                img_wnd.clear_overlay()
                img_wnd.click_coord_update()

                dst = cv.cornerHarris(img_wnd.frame, blockSize, kSize, k)
                #dst = cv.dilate(dst,None)  #only used to help see the corners when drawn on image.
                ret, dst = cv.threshold(dst,0.01*dst.max(),255,0)
                dst = np.uint8(dst)

                # find centroids
                ret, labels, stats, centroids = cv.connectedComponentsWithStats(dst)

                # define the criteria to stop and refine the corners
                criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 100, 0.001)
                corners = cv.cornerSubPix(img_wnd.frame,np.float32(centroids),(5,5),(-1,-1),criteria)

                corners = np.int0(corners)  #make corners into integer values

                # Draw corners
                for corner in range(len(corners)):
                    #cv.circle(img_wnd.frame, (corners[corner][0],corners[corner][1]), 2, (255), 2)
                    dots = cv.circle(np.ones(img_wnd.size), (corners[corner][0],corners[corner][1]), 2, 0, 2)   #10, 0, 3 = radius, color, thickness
                    img_wnd.add_overlay(dots)

            # Exit if img_wnd thread killed
            if not img_wnd.thread.is_alive():
                break

    def houghline(self):

        img_wnd = Image_window(size = self.pvcam_instance.size)

        img_wnd.set_live_thread()
        img_wnd.thread.setDaemon(True)
        img_wnd.thread.start()

        # Wait for image window ready
        img_wnd.wait_window_ready()
        img_wnd.set_mouse_response()

        img_proc = Image_process()

        while True:
            # Generate random pixel map at a rate
            img_wnd.frame = self.pvcam_instance.get_frame()

            time.sleep(0.1)

            # If clicked a new coordinate and update coordinate
            if not img_wnd.click_coord == img_wnd.previous_click_coord:
                img_wnd.clear_overlay()
                img_wnd.click_coord_update()

                img_proc.load_frame(img_wnd.frame)
                img_proc.contour()
                #img_wnd.add_overlay(img_proc.img_list[-1].edge)

            #Click once to get a list of centroid points and draw arrowedline from centroid to projected points.

                arrow_pt_list = np.array([])
                centroid_list = np.array([])
                projection_list = np.array([])

                #Take window frame and invert colors for threshold imaging
                blurred_img = cv.blur(img_wnd.frame,(5,5))
                thresh_inv = cv.threshold(blurred_img, 150, 255, cv.THRESH_BINARY_INV)[1]  #150 is the best value 

                thresh_inv_edges = cv.Canny(thresh_inv, 30, 200)
                #cv.imshow('Inverse Image',thresh_inv_edges)
                #cv.waitKey(0)

                inverse_contours, inverse_hierarchy = cv.findContours(thresh_inv_edges, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
                #inverse_contours, inverse_hierarchy = cv.findContours(thresh_inv_edges, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
                inverse_hierarchy = inverse_hierarchy[0]
                print(f"Contours Hierarchy = {inverse_hierarchy}")

                # Hierarchy = [Next, Previous, First Child, Parent]
                #c[0] = inverse_contours, c[1] = inverse_hierarchy
                #Check if 'Next' location in hierarchy (c[1]) is -1 which is the outer most contour. 
                #All detected contours are not closed so there are no parent and children parameters for these contours. 
                #Locations c[2] and c[3] are set to -1
                outer_contours = [c[0] for c in zip(inverse_contours,inverse_hierarchy) if c[1][0] == -1]

                blank = np.zeros(img_wnd.frame.shape[:2], dtype=np.uint8)
                outer_contours_lines = cv.drawContours(blank, outer_contours, -1, (255,255,255), 1)
                #cv.imshow('Outer Contours Lines',outer_contours_lines)
                #cv.waitKey(0)

                #Must have edge to feed into hough_lines_intersect
                #outer_edge = cv.Canny(outer_contours_lines, 150, 200) #30 or 150 produce similar results.
                outer_edge = cv.Canny(outer_contours_lines, 30, 200)
                #cv.imshow('outer_edge',outer_edge)
                #cv.waitKey(0)

                #Draw arrow from the potential intersection of 2-4 contour lines
                img_proc.houghline_p_list,img_proc.houghline_intersect_list = img_proc.hough_lines_intersect(outer_edge)

                #print(img_proc.houghline_p_list)
                #print(img_proc.houghline_intersect_list)

                out_idx = []
                filtered_intersect_list = []

                if img_proc.houghline_p_list is not None:

                    for line in img_proc.houghline_p_list:
                        x1,y1,x2,y2 = line[0]
                        img_wnd.frame = cv.line(img_wnd.frame,(x1,y1),(x2,y2),(0,0,0),5)
                        centroid_list = np.append(centroid_list,[x1,y1])
                        centroid_list = np.append(centroid_list,[x2,y2])

                        #Get the index of the projected points that are on or near the HoughlinesP
                        for idx,point in enumerate(img_proc.houghline_intersect_list):
                            x3,y3 = point

                            #Calculate the distance of x3,y3 w.r.t. x1,y1 and x2,y2 of the line to see if it lies between these 2 points
                            check = img_proc.is_between([x1,y1],[x3,y3],[x2,y2])
                            
                            #if x3,y3 lies between these 2 points, save the index
                            if check == True:
                                out_idx = np.append(out_idx,idx)     
                            else:
                                continue
                    
                    centroid_list = np.reshape(centroid_list,[-1,2])
                    centroid_average = np.mean(centroid_list,axis=0)

                    arrow_pt_list = np.append(arrow_pt_list,[centroid_average[0],centroid_average[1]])
                    #print(f"avg centroid = {arrow_pt_list}")

                    #Exclude the projected points on or near the HoughlinesP using the index
                    #print("Unique index list:")
                    unique_out_idx = list(set(out_idx))
                    #print(unique_out_idx)

                    #remove duplicates in img_proc.houghline_intersect_list
                    filtered_intersect_list = img_proc.houghline_intersect_list.copy()               

                    if unique_out_idx is not None:
                        for index in sorted(unique_out_idx, reverse=True):
                            index = int(index)  #index must be integer to pass into ndarray
                            filtered_intersect_list = np.delete(filtered_intersect_list,index,axis=0)

                    #print("Houghlines Intersect List:")
                    #print(img_proc.houghline_intersect_list)
                    #print("Filtered Intersect List:")
                    #print(filtered_intersect_list)

                if filtered_intersect_list is not None:
                    for point in filtered_intersect_list:
                        pt_x,pt_y = point

                        #x = img_wnd.size[1]
                        #y = img_wnd.size[0]
                        if pt_x > img_wnd.size[1] or pt_y > img_wnd.size[0] or pt_x < 0 or pt_y < 0:
                                continue
                        else:
                            projection_list = np.append(projection_list,[pt_x,pt_y])

                            #Format the projected points to be drawn on the screen
                            #pt_x = int(pt_x)
                            #pt_y = int(pt_y)
                            #dots = cv.circle(img_wnd.frame,(pt_x,pt_y),radius=10,color=(0,0,0),thickness=10)

                    if not len(projection_list) == 0:
                        projection_list = np.reshape(projection_list,[-1,2])

                        k_centroid_list,k_sse_list = img_proc.k_means(projection_list)

                        for k_point in k_centroid_list:
                            k_x,k_y = k_point
                            #Format and draw the k-means cluster points on the screen
                            k_x = int(k_x) 
                            k_y = int(k_y)
                            dots = cv.circle(img_wnd.frame,(k_x,k_y),radius=10,color=(255,255,255),thickness=10)

                            img_wnd.add_overlay(dots)

                        k_centroid_list = np.reshape(k_centroid_list,[-1,2])
                        k_centroid_average = np.mean(k_centroid_list,axis=0)

                        #List all of the projected intersection points
                        #print("List of projected points")
                        #print(k_centroid_list)

                        arrow_pt_list = np.append(arrow_pt_list,[k_centroid_average[0],k_centroid_average[1]])          

                #Display arrow and location/angle
                if not len(arrow_pt_list) == 0:
                    arrow_pt_list = np.reshape(arrow_pt_list,[-1,2])
                    print(f"Arrowed Line points = {arrow_pt_list}")

                    start_point = (int(arrow_pt_list[0][0]), int(arrow_pt_list[0][1]))
                    end_point = (int(arrow_pt_list[-1][0]), int(arrow_pt_list[-1][1]))
                    img_wnd.frame = cv.arrowedLine(img_wnd.frame,start_point,end_point,(0,0,0),thickness=10)
                    img_wnd.add_overlay(img_wnd.frame)

                    #Calculate angle
                    delta_y = int(arrow_pt_list[-1][1]) - int(arrow_pt_list[0][1])
                    delta_x = int(arrow_pt_list[-1][0]) - int(arrow_pt_list[0][0])

                    angle = np.arctan2(delta_y, delta_x)

                    while angle < 0:
                        angle = angle + (2*np.pi)

                    #print(f"angle = {angle} ")  #clockwise from start point to end point
                    print(f"Clockwise angle = {np.rad2deg(angle)} deg")  #clockwise from start point to end point
                    print(f"Counter clockwise angle = {360 - np.rad2deg(angle)} deg") #counter-clockwise from start point to end point 

                    print("Direction and Line are calculated with respect to the start coordinate. Imagine a Horizon Line through the Starting Point")
                    print(f"start coord = {start_point}")
                    print(f"end coord = {end_point}")
                    print(f"Move by x = {delta_x}")
                    print(f"Move by y = {delta_y}")

                    #Rough estimate of Quadrant location based on projected intersection point.
                    #screen 1376 is in position 1 and is the X axis
                    #screen 1024 is in position 0 and is the Y axis

                    center_of_screen = [int(img_wnd.size[1]/2),int(img_wnd.size[0]/2)]
                    #print(f"center of screen = {center_of_screen}")
                    #print(f"x = {int(img_wnd.size[1])}, y={int(img_wnd.size[0])}")

                    #identify the quadrant the end point/tip is located
                    arrow_tip_x = int(arrow_pt_list[-1][0])
                    arrow_tip_y = int(arrow_pt_list[-1][1])

                    if arrow_tip_x > center_of_screen[0] and arrow_tip_y < center_of_screen[1]:
                        print(f"arrow tip x = {arrow_tip_x}, arrow tip y = {arrow_tip_y}")
                        print("Quadrant 1 (+x,+y)")
                    if arrow_tip_x < center_of_screen[0] and arrow_tip_y < center_of_screen[1]:
                        print(f"arrow tip x = {arrow_tip_x}, arrow tip y = {arrow_tip_y}")
                        print("Quadrant 2 (-x,+y)")
                    if arrow_tip_x < center_of_screen[0] and arrow_tip_y > center_of_screen[1]:
                        print(f"arrow tip x = {arrow_tip_x}, arrow tip y = {arrow_tip_y}")
                        print("Quadrant 3 (-x,-y)")
                    if arrow_tip_x > center_of_screen[0] and arrow_tip_y > center_of_screen[1]:
                        print(f"arrow tip x = {arrow_tip_x}, arrow tip y = {arrow_tip_y}")
                        print("Quadrant 4 (+x,-y)")

    def high_mag_roi(self):

        img_wnd = Image_window(size = self.pvcam_instance.size)

        img_wnd.set_live_thread()
        img_wnd.thread.setDaemon(True)
        img_wnd.thread.start()

        # Wait for image window ready
        img_wnd.wait_window_ready()
        img_wnd.set_mouse_response()

        img_proc = Image_process()

        while True:
            # Generate random pixel map at a rate
            img_wnd.frame = self.pvcam_instance.get_frame()

            time.sleep(0.1)

            # If clicked a new coordinate and update coordinate
            if not img_wnd.click_coord == img_wnd.previous_click_coord:
                img_wnd.clear_overlay()
                img_wnd.click_coord_update()

                height,width = img_wnd.frame.shape

                # [0] = 1024, [1] = 1376
                #print(height) #[0]
                #print(width)  #[1]

                ##### Denoising algorithms #####
                img_cleaned = self.non_local_means_CV()
                #img_cleaned = self.non_local_means_SK()
                #print(img_cleaned.dtype)

                r = cv.selectROI('Select ROI', img_cleaned)
                #There will be a prompt saying "Select a ROI and then press SPACE or ENTER button!"
                #"Cancel the selection process by pressing c button!"

                #The ROI should contain the probe within it. But if it does not, 
                #the HoughlinesP should be a check to see if there is a probe shape there. 
                x = r[0] #top left x
                y = r[1] #top left y
                w = r[2] #width
                h = r[3] #height

                #print(x,y,w,h)
                #x,y is your reference coordinate for the probe tip.
                #w,h is your new image crop dimensions.

                #img_crop = img_cleaned[int(r[1]):int(r[1]+r[3]),
                #                       int(r[0]):int(r[0]+r[2])]
                #Same as above
                img_crop = img_cleaned[int(y):int(y+h),
                                        int(x):int(x+w)]

                img_crop = img_crop.astype('uint8')*255
                #img_crop = cv.normalize(src=img_crop, dst=None, alpha=0, beta=255, norm_type=cv.NORM_MINMAX, dtype=cv.CV_8U)   #Same as above
                #print(img_cleaned.dtype)   #image format is float64. CV_U8C3 at this line (uint8 for 3 colors/channels)

                img_crop = cv.cvtColor(img_crop, cv.COLOR_BGR2GRAY)  #Convert from 3 colors/channels to 1. CV_U8C1
                #print(img_crop.dtype)  #image format is uint8 now.
                #cv.imshow('img crop', img_crop)
                #cv.waitKey(3)
                #print('img_crop dim')
                #print(img_crop.shape)   #img_crop[0] = height, img_crop[1] = width

                threshold = cv.threshold(img_crop,0,255,cv.THRESH_BINARY+cv.THRESH_OTSU)[1]

                kernel = cv.getStructuringElement(cv.MORPH_RECT, (3,3))
                close = cv.morphologyEx(threshold, cv.MORPH_CLOSE, kernel)
                #cv.imshow('close', close)
                #cv.waitKey(3)

                #self.project_houghline_intercept(close)

                edges = cv.Canny(close, 30, 200)
                img_proc.houghline_p_list,img_proc.houghline_intersect_list = img_proc.hough_lines_intersect(edges, line_vote_threshold = 100, min_line_length = 50, max_line_gap = 5)

                if img_proc.houghline_p_list is not None:

                    hull_thresh, maxRect, maxEllipse, points, area = img_proc.detect_probe_high_mag(close)

                    #hull_thresh.shape[0] = height = y, hull_thresh[1] = width = x
                    ROI_center_of_screen = [int(hull_thresh.shape[1]/2),int(hull_thresh.shape[0]/2)]

                    font = cv.FONT_HERSHEY_SIMPLEX 
                    fontScale = 1
                    colorText = (255,255,255)
                    thickness = 2

                    # Ellipse
                    if not maxEllipse == None:
                        cv.ellipse(hull_thresh, maxEllipse, (255,255,255), 3)
                        cv.putText(hull_thresh, 'E: '+ str(maxEllipse[2]), (ROI_center_of_screen[0],ROI_center_of_screen[1]), font, fontScale, colorText, thickness, cv.LINE_AA)      
                        print("Ellipse has angle: " + str(maxEllipse[2]) + "\n")

                    # Rotated rectangle
                        rotated_box = cv.boxPoints(maxRect)
                        rotated_box = np.intp(rotated_box) #np.intp: Integer used for indexing (same as C ssize_t; normally either int32 or int64)
                        cv.drawContours(hull_thresh, [rotated_box], 0, (255,255,255), 3)

                    else:
                        break

                    #print(box)

                    cv.imshow('Contours', hull_thresh)
                    cv.waitKey(3)


                    (x1,y1),(x2,y2), angle = img_proc.detect_probe_direction(hull_thresh, maxEllipse, points)

                    print('(x1,y1) = ', x1, y1)
                    print('(x2,y2) = ', x2, y2)
                    print('Angle = ', angle)

                    #Calculate the rectangle box 'points' closest to x2,y2 from ellipse.

                    #print(points)

                    if (np.isnan(x2)) or (np.isnan(y2)):
                        pass

                    else:

                        filtered_points = []
                        border = 10

                        for k in range(len(points)):

                            if (points[k][0] >= border) and (points[k][0] <= (int(width)-border)):
                                if (points[k][1] >= border) and (points[k][1] <= (int(height)-border)):
                                #At least one of the Box coordinates is away from an edge

                                    filtered_points = np.append(filtered_points, [points[k][0],points[k][1]])

                        filtered_points = np.reshape(filtered_points,[-1,2])

                        #print(filtered_points)

                        dist = []
                        for p in range(len(filtered_points)):
                            dist = np.append(dist, np.sqrt( ((x2-filtered_points[p][0])**2) + ((y2-filtered_points[p][1])**2) ))

                        #print(dist)

                        #Find the index with the smallest distance
                        min_sum_index = np.argmin(dist)   #dist.index(min(dist)) for lists

                        closest_rect_point_x = filtered_points[min_sum_index][0]
                        closest_rect_point_y = filtered_points[min_sum_index][1]

                        #print(closest_rect_point_x)
                        #print(closest_rect_point_y)

                        probe_tip_x = x + int(closest_rect_point_x)
                        probe_tip_y = y + int(closest_rect_point_y)

                        #print(int(img_wnd.size[1]),int(img_wnd.size[0]))
                        center_of_screen = [int(width/2),int(height/2)]
                        q1_x = int(center_of_screen[0] + (center_of_screen[0]/4))
                        q1_y = int(center_of_screen[1] - (center_of_screen[1]/4))
                        q2_x = int(center_of_screen[0] - (center_of_screen[0]/4))
                        q2_y = int(center_of_screen[1] - (center_of_screen[1]/4))
                        q3_x = int(center_of_screen[0] - (center_of_screen[0]/4))
                        q3_y = int(center_of_screen[1] + (center_of_screen[1]/4))
                        q4_x = int(center_of_screen[0] + (center_of_screen[0]/4))
                        q4_y = int(center_of_screen[1] + (center_of_screen[1]/4))

                        font = cv.FONT_HERSHEY_SIMPLEX 
                        fontScale = 1
                        colorText = (0)
                        thickness = 2

                        #Q1
                        if probe_tip_x > (int(width)/2) and probe_tip_y < (int(height)/2):
                            print("Q1 " + "\n")
                            cv.putText(img_wnd.frame, 'Q1', (q1_x,q1_y), font, fontScale, colorText, thickness, cv.LINE_AA) 

                        #Q2
                        if probe_tip_x < (int(width)/2) and probe_tip_y < (int(height)/2):
                            print("Q2" + "\n")
                            cv.putText(img_wnd.frame, 'Q2', (q2_x,q2_y), font, fontScale, colorText, thickness, cv.LINE_AA) 

                        #Q3
                        if probe_tip_x < (int(width)/2) and probe_tip_y > (int(height)/2):
                            print("Q3" + "\n")
                            cv.putText(img_wnd.frame, 'Q3', (q3_x,q3_y), font, fontScale, colorText, thickness, cv.LINE_AA) 

                        #Q4
                        if probe_tip_x > (int(width)/2) and probe_tip_y > (int(height)/2):
                            print("Q4" + "\n")
                            cv.putText(img_wnd.frame, 'Q4', (q4_x,q4_y), font, fontScale, colorText, thickness, cv.LINE_AA) 

                        #Calculate delta of x,y point to center of screen
                        #delta_x_from_center = int(probe_tip_x-center_of_screen[0])
                        #delta_y_from_center = int(probe_tip_y-center_of_screen[1])

                        #print('delta x from center')
                        #print(delta_x_from_center)
                        #print('delta y from center')
                        #print(delta_y_from_center)
                        
                        dot = cv.circle(img_wnd.frame,(probe_tip_x,probe_tip_y),radius=10,color=(255,255,255),thickness=10)
                        cv.imshow('Dot', dot)
                        cv.waitKey(3)

                        #img_wnd.add_overlay(cv.circle(img_wnd.frame,(probe_tip_x,probe_tip_y),radius=10,color=(255,255,255),thickness=10))
                        #img_wnd.add_overlay(dot)
                        #cv.destroyAllWindows()

    def filter_histogram(self, img, n=6):

        #img_uint8 = 255*img
        #img_uint8 = img_uint8.astype('uint8')
        img_uint8 = img.astype('uint8')*255
        #print(img_uint8.dtype) 

        hist = cv.calcHist([img_uint8], [0], None, [256], [0, 256])  #must use uint8 with [0, 256] range
        hist = [values[0] for values in hist]
        indices = list(range(0, 256))

        #print(hist)

        #plt.figure()
        #plt.bar(indices,hist)

        max = np.max(hist)
        #avg = np.mean(hist)
        var = np.var(hist)
        std_dev = np.sqrt(var)

        #default is 6 sigma (6 std dev)
        hist_delta = n*std_dev
        low_hist_y = max - hist_delta
        high_hist_y = max + hist_delta

        hist_thresh = []
        hist_thresh_idx = []

        idx = 0
        for y in hist:
            if y < high_hist_y:        
                if y > low_hist_y:                
                    hist_thresh_idx.append(idx)
                    hist_thresh.append(y)
            idx = idx + 1

        #plt.figure()
        #plt.bar(hist_thresh_idx, hist_thresh)

        left_x = np.min(hist_thresh_idx)
        right_x = np.max(hist_thresh_idx)

        #print('Left = ', left_x)
        #print('Right = ', right_x)

        segment = (img_uint8 > left_x) & (img_uint8 <= right_x)

        #White canvas, Black marker
        #all_segments = np.ones((img_uint8.shape[0], img_uint8.shape[1], 3))
        #all_segments[segment] = (0,0,0)  #(0,0,0) means black

        #Black canvas, White marker
        all_segments = np.zeros((img_uint8.shape[0], img_uint8.shape[1], 3))
        all_segments[segment] = (1,1,1)  #(0,0,0) means black

        #plt.figure()
        #plt.imshow(all_segments)

        #plt.show()

        return all_segments

    def non_local_means_CV(self):

        img = self.get_snapshot(self.window.window_label)
        #sigma_est = np.mean(estimate_sigma(img))
        #nlm = cv.fastNlMeansDenoising(img, None, h = 1.25*sigma_est, templateWindowSize = 7, searchWindowSize = 21)  #minimum h=3

        nlm = cv.fastNlMeansDenoising(img, None, h = 10, templateWindowSize = 7, searchWindowSize = 21)  #minimum h=3

        #cv.imshow('Non Local Means - CV', nlm)   #must be float64.
        #cv.waitKey(5)

        #img = img_as_float(img)
        #plt.figure(1)
        #plt.hist(img.flat, bins = 100, range = (0,1))

        #nlm = img_as_float(nlm)
        #plt.figure(2)
        #plt.hist(nlm.flat, bins = 100, range = (0,1))

        result = self.filter_histogram(nlm)

        return result

    def non_local_means_SK(self):

        img = self.get_snapshot(self.window.window_label)
        #sigma_est = np.mean(estimate_sigma(img))
        #nlm = denoise_nl_means(img, h = 1.25*sigma_est, fast_mode = True, patch_size = 5, patch_distance = 3)  #h=1.25* ,  patch_size = 5, patch_distance = 3

        nlm = denoise_nl_means(img, h = 10, fast_mode = True, patch_size = 5, patch_distance = 3)  #h=1.25* ,  patch_size = 5, patch_distance = 3

        #cv.imshow('Non Local Means - Sci Kit', nlm)
        #cv.waitKey(5)

        #img = img_as_float(img)
        #plt.figure(1)
        #plt.hist(img.flat, bins = 100, range = (0,1))

        #nlm = img_as_float(nlm)
        #plt.figure(2)
        #plt.hist(nlm.flat, bins = 100, range = (0,1))

        result = self.filter_histogram(nlm)

        return result

    def project_houghline_intercept(self, input, line_vote_thresh = 100, min_line_len_thresh = 50, max_line_gap_thresh = 5):

        img_proc = Image_process()

        height, width = input.shape[:2]
        print(height, width)

        img_proc.load_frame(input)
        #avg_contours, point_list = img_proc.contour()
        #self.fit_piecewise_linear(img_proc.img_list[-1].edge)

    #Click once to get a list of centroid points and draw arrowedline from centroid to projected points.

        arrow_pt_list = np.array([])
        centroid_list = np.array([])
        projection_list = np.array([])

        edges = cv.Canny(input, 30, 200)
        #cv.imshow('Image',edges)
        #cv.waitKey(0)

        #Draw arrow from the potential intersection of 2-4 contour lines
        #img_proc.houghline_p_list,img_proc.houghline_intersect_list = img_proc.hough_lines_intersect(edges)
        img_proc.houghline_p_list,img_proc.houghline_intersect_list = img_proc.hough_lines_intersect(edges, line_vote_threshold = line_vote_thresh, min_line_length = min_line_len_thresh, max_line_gap = max_line_gap_thresh)

        print(img_proc.houghline_p_list)
        print(img_proc.houghline_intersect_list)

        out_idx = []
        filtered_intersect_list = []

        if img_proc.houghline_p_list is not None:

            for line in img_proc.houghline_p_list:
                x1,y1,x2,y2 = line[0]
                edges = cv.line(edges,(x1,y1),(x2,y2),(255,255,255),5)
                centroid_list = np.append(centroid_list,[x1,y1])
                centroid_list = np.append(centroid_list,[x2,y2])

                #Get the index of the projected points that are on or near the HoughlinesP
                for idx,point in enumerate(img_proc.houghline_intersect_list):
                    x3,y3 = point

                    #Calculate the distance of x3,y3 w.r.t. x1,y1 and x2,y2 of the line to see if it lies between these 2 points
                    check = img_proc.is_between([x1,y1],[x3,y3],[x2,y2])
                    
                    #if x3,y3 lies between these 2 points, save the index
                    if check == True:
                        out_idx = np.append(out_idx,idx)     
                    else:
                        continue
            
            centroid_list = np.reshape(centroid_list,[-1,2])
            centroid_average = np.mean(centroid_list,axis=0)

            arrow_pt_list = np.append(arrow_pt_list,[centroid_average[0],centroid_average[1]])
            #print(f"avg centroid = {arrow_pt_list}")

            #Exclude the projected points on or near the HoughlinesP using the index
            #print("Unique index list:")
            unique_out_idx = list(set(out_idx))
            #print(unique_out_idx)

            #remove duplicates in img_proc.houghline_intersect_list
            filtered_intersect_list = img_proc.houghline_intersect_list.copy()               

            if unique_out_idx is not None:
                for index in sorted(unique_out_idx, reverse=True):
                    index = int(index)  #index must be integer to pass into ndarray
                    filtered_intersect_list = np.delete(filtered_intersect_list,index,axis=0)

            print("Houghlines Intersect List:")
            print(img_proc.houghline_intersect_list)
            print("Filtered Intersect List:")
            print(filtered_intersect_list)

        cv.imshow('Edges with hough lines P',edges)
        cv.waitKey(5)

        if filtered_intersect_list is not None:
            for point in filtered_intersect_list:
                pt_x,pt_y = point

                #x = img_wnd.size[1]
                #y = img_wnd.size[0]
                if pt_x > width or pt_y > height or pt_x < 0 or pt_y < 0:
                        continue
                else:
                    projection_list = np.append(projection_list,[pt_x,pt_y])

                    #Format the projected points to be drawn on the screen
                    #pt_x = int(pt_x)
                    #pt_y = int(pt_y)
                    #dots = cv.circle(img_wnd.frame,(pt_x,pt_y),radius=10,color=(0,0,0),thickness=10)

            if not len(projection_list) == 0:
                projection_list = np.reshape(projection_list,[-1,2])

                k_centroid_list,k_sse_list = img_proc.k_means(projection_list)

                for k_point in k_centroid_list:
                    k_x,k_y = k_point
                    #Format and draw the k-means cluster points on the screen
                    k_x = int(k_x) 
                    k_y = int(k_y)
                    dots = cv.circle(edges,(k_x,k_y),radius=10,color=(255,255,255),thickness=10)

                k_centroid_list = np.reshape(k_centroid_list,[-1,2])
                k_centroid_average = np.mean(k_centroid_list,axis=0)

                #List all of the projected intersection points
                #print("List of projected points")
                #print(k_centroid_list)

                arrow_pt_list = np.append(arrow_pt_list,[k_centroid_average[0],k_centroid_average[1]])          

        cv.imshow('Edges with Dots',dots)
        cv.waitKey(5)

    def clean_image(self):

        ##### Denoising algorithms #####
        img_cleaned = self.non_local_means_CV()
        #img_cleaned = self.non_local_means_SK()
        #print(img_cleaned.dtype)

        img_cleaned = img_cleaned.astype('uint8')*255
        #img_crop = cv.normalize(src=img_crop, dst=None, alpha=0, beta=255, norm_type=cv.NORM_MINMAX, dtype=cv.CV_8U)   #Same as above
        #print(img_cleaned.dtype)   #image format is float64. CV_U8C3 at this line (uint8 for 3 colors/channels)

        img_cleaned = cv.cvtColor(img_cleaned, cv.COLOR_BGR2GRAY)  #Convert from 3 colors/channels to 1. CV_U8C1
        #print(img_crop.dtype)  #image format is uint8 now.
        #cv.imshow('img_cleaned', img_cleaned)
        #cv.waitKey(0)

        threshold = cv.threshold(img_cleaned,0,255,cv.THRESH_BINARY+cv.THRESH_OTSU)[1]

        kernel = cv.getStructuringElement(cv.MORPH_RECT, (3,3))
        close = cv.morphologyEx(threshold, cv.MORPH_CLOSE, kernel)

        return close

    def high_mag_into_water(self):
        img_proc = Image_process()

        self.update_coordinates()

        cnt = 0

        while True:

            close = self.clean_image()

            edges = cv.Canny(close, 30, 200)
            img_proc.houghline_p_list,img_proc.houghline_intersect_list = img_proc.hough_lines_intersect(edges, line_vote_threshold = 200, min_line_length = 50, max_line_gap = 5)

            #print(img_proc.houghline_p_list)

            #for line in img_proc.houghline_p_list:
            #    x1,y1,x2,y2 = line[0]
            #    edges = cv.line(edges,(x1,y1),(x2,y2),(255,255,255),5)

            #cv.imshow('Edges with hough lines P',edges)
            #cv.waitKey(0)

            #Focus down until you hit water and see the probe
            if img_proc.houghline_p_list is None:

                cnt = cnt + 1
                print(f"iteration = {cnt}")

                if self.slicescope_instance.z < self.slicescope_instance.high_mag_z_min:     #1000_00-2000_00
                    break

                if self.slicescope_instance.z > 4000_00:     #2000_00-3000_00
                    step = 1000_00
                elif self.slicescope_instance.z > 2000_00:
                    step = 100_00
                else:
                    step = 10_00

                self.slicescope_instance.moveRelative(self.slicescope_instance.x,self.slicescope_instance.y,self.slicescope_instance.z,0,0,-int(step))

                #update the video feed in window_label
                self.window.window_label.update()

            else:

                #Save this coordinate to be used in high_mag_on_tip as upperbound

                if img_proc.houghline_p_list.shape[0] > 1:

                    self.slicescope_instance.high_mag_z_max = self.slicescope_instance.z
                    print(f'Slicescope High Mag Z Max = {self.slicescope_instance.high_mag_z_max}')

                    hull_thresh, maxRect, maxEllipse, points, area = img_proc.detect_probe_high_mag(close)

                    #update the video feed in window_label
                    self.window.window_label.update()

                    break

    def minor_high_mag_on_probe(self):

        img_proc = Image_process()

        self.update_coordinates()

        self.slicescope_instance.high_mag_z_max = self.slicescope_instance.z
        print(f'Slicescope High Mag Z Max = {self.slicescope_instance.high_mag_z_max}')

        step = 10_00

        close = self.clean_image()

        edges = cv.Canny(close, 30, 200)
        img_proc.houghline_p_list,img_proc.houghline_intersect_list = img_proc.hough_lines_intersect(edges, line_vote_threshold = 200, min_line_length = 50, max_line_gap = 5)

        if img_proc.houghline_p_list is None:
            pass

        elif np.isnan(img_proc.houghline_p_list).any() == True:
            pass

        else:

            num_segments = 2

            for cnt in range(num_segments):

                print(f"iteration = {cnt+1}")

                self.slicescope_instance.moveRelative(self.slicescope_instance.x,self.slicescope_instance.y,self.slicescope_instance.z,0,0,-int(step))

                #update the video feed in window_label
                self.window.window_label.update()

                if self.slicescope_instance.z < self.slicescope_instance.high_mag_z_min:
                    #update the video feed in window_label
                    self.window.window_label.update()

                    break

        return img_proc.houghline_p_list

    def high_mag_on_probe(self):

        img_proc = Image_process()

        self.update_coordinates()

        self.slicescope_instance.high_mag_z_max = self.slicescope_instance.z
        print(f'Slicescope High Mag Z Max = {self.slicescope_instance.high_mag_z_max}')

        step = 10_00

        close = self.clean_image()

        edges = cv.Canny(close, 30, 200)
        img_proc.houghline_p_list,img_proc.houghline_intersect_list = img_proc.hough_lines_intersect(edges, line_vote_threshold = 200, min_line_length = 50, max_line_gap = 5)

        if img_proc.houghline_p_list is None:
            pass

        elif np.isnan(img_proc.houghline_p_list).any() == True:
            pass

        else:

            num_segments = 8

            for cnt in range(num_segments):

                print(f"iteration = {cnt+1}")

                self.slicescope_instance.moveRelative(self.slicescope_instance.x,self.slicescope_instance.y,self.slicescope_instance.z,0,0,-int(step))

                #update the video feed in window_label
                self.window.window_label.update()

                if self.slicescope_instance.z < self.slicescope_instance.high_mag_z_min:
                    #update the video feed in window_label
                    self.window.window_label.update()

                    break

        return img_proc.houghline_p_list


    def high_mag_on_tip(self):

        img_proc = Image_process()

        self.update_coordinates()

        #Setup figure properties for matplotlib
        fig = plt.figure()
        ax = fig.add_subplot(1, 1, 1)
        ax.set(title = 'Convergence on the probe tip', xlabel = 'Detected lines index', ylabel = 'Number of lines')
        plt.show(block=False)

        #Move slicescope up until probe is in focus
        #Calculate rolling average to find the minimum number of corners. This is when the probe is in focus.
        xs = []
        ys = []
        moving_averages = []
        focus_height = []

        #thread = Thread(target = self.plot_graph(ax, xs, moving_averages))
        thread = Thread(target = self.plot_graph(ax, xs, ys))
        thread.setDaemon(True)
        thread.start()

        cnt = 0
        line_cnt = 0
        step = 1_00

        while True:

            close = self.clean_image()

            edges = cv.Canny(close, 30, 200)
            img_proc.houghline_p_list,img_proc.houghline_intersect_list = img_proc.hough_lines_intersect(edges, line_vote_threshold = 200, min_line_length = 50, max_line_gap = 5)

            #Focus up to the tip
            if img_proc.houghline_p_list is None:
                cnt = cnt + 1
                print(f"iteration = {cnt}")

                self.slicescope_instance.moveRelative(self.slicescope_instance.x,self.slicescope_instance.y,self.slicescope_instance.z,0,0,int(step))

                #update the video feed in window_label
                self.window.window_label.update()

                if self.slicescope_instance.z > int(1.1*self.slicescope_instance.high_mag_z_max):

                    #update the video feed in window_label
                    self.window.window_label.update()

                    break

            elif np.isnan(img_proc.houghline_p_list).any() == True:

                cnt = cnt + 1
                print(f"iteration = {cnt}")

                self.slicescope_instance.moveRelative(self.slicescope_instance.x,self.slicescope_instance.y,self.slicescope_instance.z,0,0,int(step))

                #update the video feed in window_label
                self.window.window_label.update()

                if self.slicescope_instance.z > int(1.1*self.slicescope_instance.high_mag_z_max):

                    #update the video feed in window_label
                    self.window.window_label.update()

                    break

            else:

                cnt = cnt + 1
                print(f"iteration = {cnt}")

                self.slicescope_instance.moveRelative(self.slicescope_instance.x,self.slicescope_instance.y,self.slicescope_instance.z,0,0,int(step))

                #update the video feed in window_label
                self.window.window_label.update()

                line_cnt = line_cnt + 1
                print(f"line iteration = {line_cnt}")

                num_lines = img_proc.houghline_p_list.shape[0]

                xs.append(line_cnt)
                ys.append(num_lines)
                focus_height.append(self.slicescope_instance.z)

                self.plot_graph(ax, xs, ys)
                plt.pause(0.001)

                '''
                window_size = 5

                if len(ys) < window_size:
                    moving_averages.append(np.nan)
                else:
                    #Calculate the average of current window
                    window_average = round(np.sum(ys[-window_size:])/window_size)
                    moving_averages.append(window_average)

                #Plot using matplotlib
                #These codes work to constantly update the plot in real time, but is slow.
                #Moving averages real time graph
                self.plot_graph(ax, xs, moving_averages)
                plt.pause(0.001)
                '''

                if self.slicescope_instance.z > self.slicescope_instance.high_mag_z_max:

                    #Find the first peak in the number of lines and use index to move slicescope to appropriate focus height
                    peaks,_ = find_peaks(ys, height=0)
                    peak_tip_index = peaks[0]
                    tip_in_focus_height = focus_height[peak_tip_index]
                    self.slicescope_instance.moveAbsolute(self.slicescope_instance.x,self.slicescope_instance.y,int(tip_in_focus_height))

                    #Find the maximum number of lines and use index to move slicescope to appropriate focus height
                    #max_convergence_index = ys.index(np.nanmax(ys))
                    #tip_in_focus_height = focus_height[max_convergence_index]
                    #self.slicescope_instance.moveAbsolute(self.slicescope_instance.x,self.slicescope_instance.y,int(tip_in_focus_height))

                    print(f"Tip in Focus - Slicescope Z = {tip_in_focus_height}")

                    #then go into detecting probe direction and tip

                    close = self.clean_image()

                    #continue from this line in the code below
                    hull_thresh, maxRect, maxEllipse, points, area = img_proc.detect_probe_high_mag(close)


                    #hull_thresh.shape[0] = height = y, hull_thresh[1] = width = x
                    #height = hull_thresh.shape[0]
                    #width = hull_thresh.shape[1]

                    (x1,y1),(x2,y2), angle = img_proc.detect_probe_direction(hull_thresh, maxEllipse, points)

                    #Calculate the rectangle box 'points' closest to x2,y2 from ellipse.

                    #print(points)

                    if (np.isnan(x2)) or (np.isnan(y2)):
                        pass

                    else:

                        print('(x1,y1) = ', x1, y1)
                        print('(x2,y2) = ', x2, y2)
                        print('Angle = ', angle)

                        img_wnd = Image_window(size = self.pvcam_instance.size)
                        img_wnd.frame = self.pvcam_instance.get_frame()
                        img_wnd.clear_overlay()

                        height = img_wnd.frame.shape[0]
                        width = img_wnd.frame.shape[1]

                        filtered_points = []
                        border = 10

                        for k in range(len(points)):

                            if (points[k][0] >= border) and (points[k][0] <= (int(width)-border)):
                                if (points[k][1] >= border) and (points[k][1] <= (int(height)-border)):
                                #At least one of the Box coordinates is away from an edge

                                    filtered_points = np.append(filtered_points, [points[k][0],points[k][1]])

                        filtered_points = np.reshape(filtered_points,[-1,2])

                        #print(filtered_points)

                        dist = []
                        for p in range(len(filtered_points)):
                            dist = np.append(dist, np.sqrt( ((x2-filtered_points[p][0])**2) + ((y2-filtered_points[p][1])**2) ))

                        #print(dist)

                        #Find the index with the smallest distance
                        min_sum_index = np.argmin(dist)   #dist.index(min(dist)) for lists

                        closest_rect_point_x = filtered_points[min_sum_index][0]
                        closest_rect_point_y = filtered_points[min_sum_index][1]

                        probe_tip_x = int(closest_rect_point_x)
                        probe_tip_y = int(closest_rect_point_y)


                        #Draw probe tip coordinate on window

                        dot = cv.circle(img_wnd.frame,(probe_tip_x,probe_tip_y),radius=10,color=(255),thickness=10)
                        dot_copy = dot.copy()
                        dot_arrow = cv.arrowedLine(dot_copy,(x1,y1),(x2,y2),color=(0),thickness=10)

                        font = cv.FONT_HERSHEY_SIMPLEX
                        fontScale = 1
                        color = (0)
                        thickness = 2

                        org1 = (0,100)
                        text1 = 'Probe tip coordinate: (' + str(probe_tip_x)+ ', ' + str(probe_tip_y) + ')'
                        dot = cv.putText(dot, text1, org1, font, fontScale, color, thickness, cv.LINE_AA, False)
                        dot_arrow = cv.putText(dot_arrow, text1, org1, font, fontScale, color, thickness, cv.LINE_AA, False)

                        format_angle = format(angle, '0.2f')
                        org2 = (0,150)
                        text2 = 'Probe angle w.r.t. Y-axis: ' + str(format_angle) + ' deg'
                        dot = cv.putText(dot, text2, org2, font, fontScale, color, thickness, cv.LINE_AA, False)
                        dot_arrow = cv.putText(dot_arrow, text2, org2, font, fontScale, color, thickness, cv.LINE_AA, False)

                        #Show probe tip in separate window
                        cv.imshow('Probe tip', dot)
                        cv.imshow('Probe direction', dot_arrow)
                        cv.waitKey(3)

                        #Save image to file
                        cv.imwrite('./probe_tip.png', dot)
                        cv.imwrite('./probe_direction.png', dot_arrow)

                        #Window_label is now live again. Save the coordinates for Slicescope and Patchstar.

                        self.update_coordinates()

                        print(f'Slicescope position - x: {self.slicescope_instance.x}')
                        print(f'Slicescope position - y: {self.slicescope_instance.y}')
                        print(f'Slicescope position - z: {self.slicescope_instance.z}')

                        self.slicescope_instance.x_tip = self.slicescope_instance.x
                        self.slicescope_instance.y_tip = self.slicescope_instance.y
                        self.slicescope_instance.z_tip = self.slicescope_instance.z

                        print(f'Micromanipulator position - x: {self.patchstar_instance.x}')
                        print(f'Micromanipulator position - y: {self.patchstar_instance.y}')
                        print(f'Micromanipulator position - z: {self.patchstar_instance.z}')
                        print(f'Micromanipulator position - a: {self.patchstar_instance.a}')

                        self.patchstar_instance.tip_coord_x = probe_tip_x
                        self.patchstar_instance.tip_coord_y = probe_tip_y
                        self.patchstar_instance.tip_angle = angle

                        print(f'Probe tip coordinate = ({self.patchstar_instance.tip_coord_x} , {self.patchstar_instance.tip_coord_y})')
                        print(f'Probe tip angle = {self.patchstar_instance.tip_angle}')

                        self.patchstar_instance.x_tip = self.patchstar_instance.x
                        self.patchstar_instance.y_tip = self.patchstar_instance.y
                        self.patchstar_instance.z_tip = self.patchstar_instance.z
                        self.patchstar_instance.a_tip = self.patchstar_instance.a

                        break

                else:

                    continue

    def high_mag_autofocus(self):

        #High mag autofocus can use img_proc.houghline_p_list == None as the parameter.
        #Steps are similar to isNan for focus down and focus up for low mag. Just use img_proc.houghline_p_list from hough lines intersect!
        #Go down and detect from None to Not None.
        #Go down further and detect from Not None to None.
        #Go up and detect from None to Not None.

        #self.high_mag_into_water()

        #step = 50_00
        #self.slicescope_instance.moveRelative(self.slicescope_instance.x,self.slicescope_instance.y,self.slicescope_instance.z,0,0,step)
        #self.slicescope_instance.moveUp(self.slicescope_instance.z,step)

        self.update_coordinates()

        nan_none_count = 0

        lines = self.high_mag_on_probe()

        while lines is None:

            nan_none_count = nan_none_count + 1

            self.slicescope_instance.moveRelative(self.slicescope_instance.x,self.slicescope_instance.y,self.slicescope_instance.z,0,0,-int(10_00))

            #update the video feed in window_label
            self.window.window_label.update()

            lines = self.high_mag_on_probe()

            if lines is not None:
                break

            if nan_none_count > 3:
                break

        while np.isnan(lines).any() == True:

            nan_none_count = nan_none_count + 1

            self.slicescope_instance.moveRelative(self.slicescope_instance.x,self.slicescope_instance.y,self.slicescope_instance.z,0,0,-int(10_00))

            #update the video feed in window_label
            self.window.window_label.update()

            lines = self.high_mag_on_probe()

            if np.isnan(lines).any() == False:
                break

            if nan_none_count > 3:
                break

        #update the video feed in window_label
        self.window.window_label.update()

        self.high_mag_on_tip()

    def minor_high_mag_autofocus(self):

        #High mag autofocus can use img_proc.houghline_p_list == None as the parameter.
        #Steps are similar to isNan for focus down and focus up for low mag. Just use img_proc.houghline_p_list from hough lines intersect!
        #Go down and detect from None to Not None.
        #Go down further and detect from Not None to None.
        #Go up and detect from None to Not None.

        #self.high_mag_into_water()

        #step = 50_00
        #self.slicescope_instance.moveRelative(self.slicescope_instance.x,self.slicescope_instance.y,self.slicescope_instance.z,0,0,step)
        #self.slicescope_instance.moveUp(self.slicescope_instance.z,step)

        self.update_coordinates()

        nan_none_count = 0

        lines = self.minor_high_mag_on_probe()

        while lines is None:

            nan_none_count = nan_none_count + 1

            self.slicescope_instance.moveRelative(self.slicescope_instance.x,self.slicescope_instance.y,self.slicescope_instance.z,0,0,-int(10_00))

            #update the video feed in window_label
            self.window.window_label.update()

            lines = self.minor_high_mag_on_probe()

            if lines is not None:
                break

            if nan_none_count > 3:
                break

        while np.isnan(lines).any() == True:

            nan_none_count = nan_none_count + 1

            self.slicescope_instance.moveRelative(self.slicescope_instance.x,self.slicescope_instance.y,self.slicescope_instance.z,0,0,-int(10_00))

            #update the video feed in window_label
            self.window.window_label.update()

            lines = self.minor_high_mag_on_probe()

            if np.isnan(lines).any() == False:
                break

            if nan_none_count > 3:
                break

        #update the video feed in window_label
        self.window.window_label.update()

        self.high_mag_on_tip()


    def heka_iv_monitor(self):

        self.batch_flag = True

        #User opens Patchmaster Next and loads their configuration. Make sure b is the GLOBAL shortcut in Patchmaster Next

        x = []
        y = []

        #Setup figure properties for matplotlib
        fig = plt.figure()
        ax = fig.add_subplot(1, 1, 1)
        ax.set(title = 'Measured Impedance Z of Probe', xlabel = 'Index', ylabel = 'Impedance Z (Ohms)')
        plt.show(block=False)

        thread = Thread(target = self.plot_graph(ax, x, y))
        thread.setDaemon(True)
        thread.start()

        i_mon = []
        v_mon = []
        z_mon = []

        #Model cell: 10Mohm -> 7.5e5
        #Model cell: floating -> 5.5e7
        #Model cell: 500Mohm ->  2e7

        #Model cell values are 2 orders of magnitude different when not using a Test Pulse. 
        #This is enough for our measurements to differentiate the Seal vs Patch modes.

        #PM Next file names
        #Mode Sender/Polling Sender: Sender.command
        #Mode Receiver/Polling Receiver: Receiv.answers

        with open(r'C:\\Users\\Lab\\Documents\\HEKA\\PM Next\\Batch\\Sender.command','wt') as fw:
            fw.write('0\n')

        time.sleep(0.1)


        #Run batch_comm function which will ALT tab to the open window (bring the program with the name PATCHMASTER NEXT to the foreground).
        Batch()

        time.sleep(0.1)

        cmd_idx = 1

        #File must be written to with only 1 command index and instruction every iteration. Each polling read will be with 1 set of instructions.
        #Must use Polling Receiver to ensure command index is accurate.

        while self.batch_flag:

            with open(r'C:\\Users\\Lab\\Documents\\HEKA\\PM Next\\Batch\\Sender.command','wt') as fw:
                line = [f'{cmd_idx}\n','Heartbeat\n','GetParameters Param-0, Param-1\n']
                fw.writelines(line)

            time.sleep(0.01)

            with open(r'C:\\Users\\Lab\\Documents\\HEKA\\PM Next\\Batch\\Receiv.answers','rt') as fr:

                content_list = list(fr)

                if len(content_list) == 3:

                    line_segment = content_list[-1].strip().split(',')
                    #v_mon = line_segment[-1]

                    line_segment_split = line_segment[0].split(' ')

                    line_segment_split[-1] = line_segment_split[-1].strip()

                    #i_mon = line_segment_split[-1]

                    z_mon.append(float(line_segment[-1]) / float(line_segment_split[-1]))

                    x.append(cmd_idx)
                    y.append(float(line_segment[-1]) / float(line_segment_split[-1]))

            self.plot_graph(ax, x, y)
            plt.pause(0.01)

            cmd_idx = cmd_idx + 1


    def heka_loop_monitor(self):

        self.batch_flag = True

        #User opens Patchmaster Next and loads their configuration. Make sure b is the GLOBAL shortcut in Patchmaster Next

        x = []
        y = []
        z_mon = []

        #Setup figure properties for matplotlib
        fig = plt.figure()
        ax = fig.add_subplot(1, 1, 1)
        ax.set(title = 'Measured Impedance Z of Probe', xlabel = 'Index', ylabel = 'Impedance Z (Ohms)', ylim = (0,1e9))
        plt.show(block=False)

        thread = Thread(target = self.plot_graph(ax, x, y))
        thread.setDaemon(True)
        thread.start()

        #PM Next file names
        #Mode Sender/Polling Sender: Sender.command
        #Mode Receiver/Polling Receiver: Receiv.answers

        with open(r'C:\\Users\\Lab\\Documents\\HEKA\\PM Next\\Batch\\Sender.command','wt') as fw:
            fw.write('0\n')

        time.sleep(0.1)

        #Run batch_comm function which will ALT tab to the open window (bring the program with the name PATCHMASTER NEXT to the foreground).
        Batch()

        time.sleep(0.1)

        with open(r'C:\\Users\\Lab\\Documents\\HEKA\\PM Next\\Batch\\Sender.command','wt') as fw:

            fw.write('1\n')
            file_path = 'OpenProtFile C:\\Users\\Lab\\Documents\\HEKA\\PM Next\\DefaultFiles\\Setup_Protocols\\SetupProt_Epc10.pro'
            fw.write(str(file_path) + '\n')
            fw.write('ExecuteProtocol Loop\n')

        time.sleep(0.1)

        with open(r'C:\\Users\\Lab\\Documents\\HEKA\\PM Next\\Batch\\Receiv.answers','rt') as fr:
            fr.readlines()

        time.sleep(0.1)

        cmd_idx = 2

        #File must be written to with only 1 command index and instruction every iteration. Each polling read will be with 1 set of instructions.
        #Must use Polling Receiver to ensure command index is accurate.

        while self.batch_flag:

            with open(r'C:\\Users\\Lab\\Documents\\HEKA\\PM Next\\Batch\\Sender.command','wt') as fw:

                line = [f'{cmd_idx}\n','Heartbeat\n','GetParameters Param-10\n']
                fw.writelines(line)

            time.sleep(0.01)

            with open(r'C:\\Users\\Lab\\Documents\\HEKA\\PM Next\\Batch\\Receiv.answers','rt') as fr:

                content_list = list(fr)

                if len(content_list) == 3:

                    line_segment = content_list[-1].strip().split(' ')
                    #z_mon = line_segment[-1]

                    z_mon.append(float(line_segment[-1]))

                    x.append(cmd_idx)
                    y.append(float(line_segment[-1]))

            self.plot_graph(ax, x, y)
            plt.pause(0.01)

            cmd_idx = cmd_idx + 1


    def heka_Rpip_monitor(self):

        self.batch_flag = True

        #User opens Patchmaster Next and loads their configuration. Make sure b is the GLOBAL shortcut in Patchmaster Next

        x = []
        y = []
        z_mon = []

        #Setup figure properties for matplotlib
        fig = plt.figure()
        ax = fig.add_subplot(1, 1, 1)
        ax.set(title = 'Measured Impedance Z of Probe', xlabel = 'Index', ylabel = 'Impedance Z (Ohms)', ylim = (0,1e9))
        plt.show(block=False)

        thread = Thread(target = self.plot_graph(ax, x, y))
        thread.setDaemon(True)
        thread.start()

        #PM Next file names
        #Mode Sender/Polling Sender: Sender.command
        #Mode Receiver/Polling Receiver: Receiv.answers

        with open(r'C:\\Users\\Lab\\Documents\\HEKA\\PM Next\\Batch\\Sender.command','wt') as fw:
            fw.write('0\n')

        time.sleep(0.1)

        #Run batch_comm function which will ALT tab to the open window (bring the program with the name PATCHMASTER NEXT to the foreground).
        Batch()

        time.sleep(0.1)

        cmd_idx = 1

        #File must be written to with only 1 command index and instruction every iteration. Each polling read will be with 1 set of instructions.
        #Must use Polling Receiver to ensure command index is accurate.

        while self.batch_flag:

            with open(r'C:\\Users\\Lab\\Documents\\HEKA\\PM Next\\Batch\\Sender.command','wt') as fw:
                fw.write(f'{cmd_idx}\n')
                file_path = 'OpenProtFile C:\\Users\\Lab\\Documents\\HEKA\\PM Next\\DefaultFiles\\Setup_Protocols\\SetupProt_Epc10.pro'
                fw.write(str(file_path) + '\n')
                fw.write('ExecuteProtocol SaveRpip\n')

                line = ['Heartbeat\n','GetParameters Param-10\n']
                fw.writelines(line)

            time.sleep(0.01)

            with open(r'C:\\Users\\Lab\\Documents\\HEKA\\PM Next\\Batch\\Receiv.answers','rt') as fr:

                content_list = list(fr)

                if len(content_list) == 5:

                    line_segment = content_list[-1].strip().split(' ')
                    #z_mon = line_segment[-1]

                    z_mon.append(float(line_segment[-1]))

                    x.append(cmd_idx)
                    y.append(float(line_segment[-1]))

            self.plot_graph(ax, x, y)
            plt.pause(0.01)

            cmd_idx = cmd_idx + 1


    def stop_batch(self):
        Batch()
        self.batch_flag = False