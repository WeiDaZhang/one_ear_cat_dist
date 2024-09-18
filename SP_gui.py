#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tues July 2 12:18:00 2024

@author: Dr. Vincent Lee
Bioptix Ltd.

"""

#Classes
from serial_port_tools import get_serial_ports

from slicescope import Slicescope
from condenser import Condenser
from patchstar import Patchstar
from pvcam import PVCAM
from heka import Heka
from batch import Batch
#from SP_tkinter import App

from image_process import Image_process
from threading import Thread
from distance_from_focus import *
from threshold_edge_contour import *
from probe_search import *
from live_update_funcs import *

import time
import random
import collections

import numpy as np
import cv2 as cv
import matplotlib.pyplot as plt
from coordinate_calib import *
from coordinate_trans import *

class SP:

    def __init__ (self):

        self.slicescope = []
        self.condenser = []
        self.patchstar_list = []
        self.cam = []
        self.heka = []
        self.batch = []

        self.list_port_info = get_serial_ports()

        #use threads to update video and coordinates for slicescope and patchstar(s)
        self.slicescope_coordinates = []

        self.patchstar_num = 0
        self.patchstar_list_coordinates = []
        self.patchstar_list_slicescope_coordinates = []
        self.patchstar_curr_idx = 0

        self.frame = []
        self.image_stack = []
        self.image_stack_coord = []
        self.contrast_circular_buffer = collections.deque(maxlen=5)
        self.img_proc = Image_process()

        self.low_mag_focal_plane_2_high_mag_surf_margin = -100_00
        self.low_mag_focal_plane_2_high_mag_surf = -3350_00   #working distance
        self.low_mag_2_high_mag_focal_plane = 98_00

        #self.low_mag_focal_plane_2_surf = -2500_00
        #self.low_mag_high_mag_delta = -850_00

        self.high_mag_centre_on_low_mag = [586,566]  #New: [586,566] [584,566] [587,566] / Previous: [604, 590][605, 595][605, 589][603, 589][601, 587]
        self.high_mag_magnification = 40
        self.low_mag_magnification = 4

        self.gui_img_stack = []
        self.gui_patchstar_coordinates = []
        self.gui_slicescope_coordinates = []
        self.gui_img_burst_interval = 50    #milliseconds

        self.live_flag = False

        self.target_list = []

        self.mouse_click_coord = {"x": None, "y": None, "update": False}
        self.b_coordinate_reset = False
        self.low_mag_cal_exp_time = 6
        self.high_mag_cal_exp_time = 10

    def initialize(self, port_list=['COM3', 'COM4', 'COM7', 'COM8']):
        #   port_list   string list     port_list[0] is the port for slicescope
        #                               port_list[1] is the port for condenser
        #                               port_list[2:end] are ports for patchstars

        self.slicescope = Slicescope(port_list[0])
        self.condenser = Condenser(port_list[1])
        for idx in range(2, len(port_list)):
            self.patchstar_list.append(Patchstar(port_list[idx]))
            self.patchstar_list_coordinates.append([])
            self.patchstar_list_slicescope_coordinates.append([])
            self.gui_patchstar_coordinates.append([])
            self.patchstar_num += 1

            #Set top speed for patchstars to 4000 micron/second, not hundredths of microns/second
            self.patchstar_list[-1].set_top_speed(4000)
            self.patchstar_list[-1].check_top_speed()

        #Set top speed for slicescope to 4000 micron/second, not hundredths of microns/second
        self.slicescope.set_top_speed(4000)
        self.slicescope.check_top_speed()

        self.cam = PVCAM()
        self.heka = Heka()
        self.batch = Batch()
        config_lu = LiveUpdateConfig()
        config_lu.cross = True
        config_lu.sls_flag = False
        config_lu.pts_flags = [False]*self.patchstar_num
        config_lu.show_result = False
        start_live_update(self, config_lu)
        if self.b_coordinate_reset:
            self.global_coordinate_reset()

        for idx in range(self.patchstar_num):
            #Move probe away in the max x and z directions
            self.patchstar_list[idx].moveAbsolute(  self.patchstar_list[idx].x_max,
                                                        self.patchstar_list[idx].y_origin,
                                                        self.patchstar_list[idx].z_max)

    def configure(self, **kwargs):
        for input_attr in kwargs:
            if hasattr(self, input_attr):
                setattr(self, input_attr, kwargs[input_attr])
                print("Parameter {} set to {}".format(input_attr, kwargs[input_attr]))
            else:
                print("Parameter {} does not exist, setting ignored".format(input_attr))

    def global_coordinate_reset(self):

        end_live_update(self)
        config_lu = LiveUpdateConfig()
        config_lu.sls_flag = False
        config_lu.pts_flags = [False]*self.patchstar_num
        config_lu.show_result = False
        start_live_update(self, config_lu)

        #Move condenser out of the way
        self.condenser.moveRelative(self.condenser.z_origin,100000_00)

        #Run Slicescope calibration
        slicescope_thread = Thread(target = self.slicescope.reset_coord_system)
        slicescope_thread.daemon = True
        slicescope_thread.start()

        patchstar_thread = Thread(target = self.patchstar_configuration)
        patchstar_thread.daemon = True
        patchstar_thread.start()

        slicescope_thread.join()
        patchstar_thread.join()

        #Move condenser back to original position
        self.condenser.moveAbsolute(self.condenser.z_origin)

    def adjust_intensity(self):
        while True:
            exp_time = int(input("Set exp time in milliseconds:"))
            b_norm = "" == input("Press [Enter] for normalized, or anything for not normalized: ")
            self.show_intensity(exp_time, b_norm)
            if input("Press [Enter] to show intensity again, or anything else to set the exp time and quit:"):
                self.cam.set_exp_time(exp_time)
                break

    def show_intensity(self, exp_time=10, b_norm = False):
        end_live_update(self)
        config_lu = LiveUpdateConfig()
        config_lu.sls_flag = False
        config_lu.pts_flags = [False]*self.patchstar_num
        config_lu.b_raw_frame = True
        config_lu.exp_time = exp_time
        config_lu.b_normalize = b_norm
        config_lu.b_min_max = False
        start_live_update(self, config_lu)

        plt.ion()
        fig, ax = plt.subplots()
        ax.set_xlim(0, 5e3)
        ax.set_ylim(0, 1e5)
        plt.grid()
        x_data = []
        y_data = []
        line, = ax.plot(x_data, y_data)

        while plt.fignum_exists(fig.number):
            if self.gui_img_stack:
                self.img_proc.updating_histogram(self.gui_img_stack[-1], ax)
                plt.pause(0.1)

        plt.ioff()
        plt.show()
        end_live_update(self)

    def patchstar_configuration(self):

        thread = [None] * self.patchstar_num

        #Simultaneously calibrate the patchstars
        for idx in range(len(thread)):

            thread[idx] = Thread(target = self.patchstar_list[idx].reset_coord_system)
            thread[idx].daemon = True
            thread[idx].start()

        #Join the threads after the patchstars have simultaneously been calibrated
        for idx in range(len(thread)):
            thread[idx].join()

    def pseudo_probe_adjustment(self):
        end_live_update(self)

        #Move condenser away from origin position
        self.condenser.moveAbsolute(self.condenser.z_max)

        coords = list()
        #coords.append([-2360, -33201, 390273 + random.randrange(-4000_00, 4000_00)])
        #coords.append([34872, 40163, 390273 + random.randrange(-4000_00, 4000_00)])
        coords.append([16256 + random.randrange(-18000, 18000),
                       3481 + random.randrange(-36000, 36000),
                       390273 + random.randrange(-3500_00, 3500_00)])
        #coords.append([-92130, 57526, 52401 + random.randrange(-4000_00, 4000_00)])
        #coords.append([2059, -10570, 52401 + random.randrange(-4000_00, 4000_00)])
        coords.append([-47094 + random.randrange(-47000, 47000),
                       23478 + random.randrange(-34000, 34000),
                       52401 + random.randrange(-3500_00, 3500_00)])
        for idx in range(self.patchstar_num):

            #Move probe away in the max x and z directions
            self.patchstar_list[idx].moveAbsolute(self.patchstar_list[idx].x_max, self.patchstar_list[idx].y_origin,self.patchstar_list[idx].z_max)

        for idx in range(self.patchstar_num):

            #Move probe away in the max x and z directions
            self.patchstar_list[idx].moveAbsolute(self.patchstar_list[idx].x_max, self.patchstar_list[idx].y_origin,self.patchstar_list[idx].z_max)

            #Move probe away in the max x and z directions
            self.patchstar_list[idx].moveAbsolute(coords[idx][0], coords[idx][1], coords[idx][2])
            #Save coordinates for each probe
            #Prompt user to prepare sample
            config_lu = LiveUpdateConfig()
            start_live_update(self, config_lu)
            #start_live_update(self)
            input('Press ENTER to continue. Please prepare sample.')
            end_live_update(self)

            #Update the slicescope x,y,z coordinates for each patchstar
            self.slicescope.coordinates()
            self.patchstar_list_slicescope_coordinates[idx] = [self.slicescope.x, self.slicescope.y, self.slicescope.z]

            #Update each patchstar x,y,z,a coordinates
            self.patchstar_list[idx].coordinates()
            self.patchstar_list_coordinates[idx] = [self.patchstar_list[idx].x, self.patchstar_list[idx].y, self.patchstar_list[idx].z, self.patchstar_list[idx].a]
            print("User Moves PatchStar{} to Coordinates: {}, {}, {}, {}".format(idx,   self.patchstar_list[idx].x,
                                                                                        self.patchstar_list[idx].y,
                                                                                        self.patchstar_list[idx].z,
                                                                                        self.patchstar_list[idx].a))

            #Move probe away in the max x and z directions
            self.patchstar_list[idx].moveAbsolute(self.patchstar_list[idx].x_max, self.patchstar_list[idx].y_origin,self.patchstar_list[idx].z_max)


    def manual_probe_adjustment(self):
        end_live_update(self)

        self.slicescope.moveAbsolute(self.slicescope.x_origin,self.slicescope.y_origin,self.slicescope.z_origin)

        #Move condenser away from origin position
        self.condenser.moveAbsolute(self.condenser.z_max)

        for idx in range(self.patchstar_num):

            config_lu = LiveUpdateConfig()
            start_live_update(self, config_lu)
            #start_live_update(self)

            #Prompt user to move probes into camera FOV
            input(f'Manually move the patchstar{idx} into Camera FOV. Press ENTER when finished.')

            end_live_update(self)

            #Save coordinates for each probe
            #Update the slicescope x,y,z coordinates for each patchstar
            self.slicescope.coordinates()
            self.patchstar_list_slicescope_coordinates[idx] = [self.slicescope.x, self.slicescope.y, self.slicescope.z]

            #Update each patchstar x,y,z,a coordinates
            self.patchstar_list[idx].coordinates()
            self.patchstar_list_coordinates[idx] = [self.patchstar_list[idx].x, self.patchstar_list[idx].y, self.patchstar_list[idx].z, self.patchstar_list[idx].a]
            print("User Moves PatchStar{} to Coordinates: {}, {}, {}, {}".format(idx,   self.patchstar_list[idx].x,
                                                                                        self.patchstar_list[idx].y,
                                                                                        self.patchstar_list[idx].z,
                                                                                        self.patchstar_list[idx].a))

            #Move probe away in the max x and z directions
            self.patchstar_list[idx].moveAbsolute(  self.patchstar_list[idx].x_max,
                                                    self.patchstar_list_coordinates[idx][1],
                                                    self.patchstar_list[idx].z_max)

        #Prompt user to prepare sample
        input('Press ENTER to continue. Please prepare sample.')

    def pre_probe_adjustment(self):
        end_live_update(self)
        config_lu = LiveUpdateConfig()
        config_lu.sls_flag = False
        config_lu.pts_flags = [False]*self.patchstar_num
        config_lu.show_result = False
        start_live_update(self, config_lu)

        self.slicescope.moveAbsolute(self.slicescope.x_origin,self.slicescope.y_origin,self.slicescope.z_origin)
        #Save coordinates for each probe
        #Update the slicescope x,y,z coordinates for each patchstar
        self.slicescope.coordinates()
        for idx in range(self.patchstar_num):
            self.patchstar_list_slicescope_coordinates[idx] = [self.slicescope.x, self.slicescope.y, self.slicescope.z]

        #Move condenser away from origin position
        self.condenser.moveAbsolute(self.condenser.z_max)

        end_live_update(self)
        config_lu = LiveUpdateConfig()
        config_lu.sls_flag = True
        config_lu.pts_flags = [True]*self.patchstar_num
        config_lu.show_result = False
        start_live_update(self, config_lu)

    def probe_adjustment(self):
        end_live_update(self)
        config_lu = LiveUpdateConfig()
        config_lu.sls_flag = False
        config_lu.pts_flags = [False]*self.patchstar_num
        config_lu.show_result = True
        start_live_update(self, config_lu)

        #Update each patchstar x,y,z,a coordinates
        patchstar_distances = []
        for idx in range(self.patchstar_num):
            patchstar_pointstack = PointStack([])
            self.patchstar_list[idx].coordinates()
            patchstar_pointstack.append(Point([ self.patchstar_list[idx].x,
                                                self.patchstar_list[idx].y,
                                                self.patchstar_list[idx].z]))
            patchstar_pointstack.append(Point([ self.patchstar_list[idx].x_max,
                                                self.patchstar_list[idx].y_origin,
                                                self.patchstar_list[idx].z_max]))
            patchstar_distances.append(patchstar_pointstack.distance()[0,1])

        max_moved_idx = np.argmax(patchstar_distances)
        if patchstar_distances[idx] == 0:
            end_live_update(self)
            config_lu = LiveUpdateConfig()
            config_lu.sls_flag = True
            config_lu.pts_flags = [True]*self.patchstar_num
            config_lu.show_result = False
            start_live_update(self, config_lu)
            print('--------return at line 366')
            return False, "None PatchStar Move Detected."
            
        self.patchstar_list_coordinates[max_moved_idx] = [  self.patchstar_list[max_moved_idx].x,
                                                            self.patchstar_list[max_moved_idx].y,
                                                            self.patchstar_list[max_moved_idx].z,
                                                            self.patchstar_list[max_moved_idx].a]

        print("User Moves PatchStar-{} to Coordinates: {}, {}, {}, {}".format(  max_moved_idx,
                                                                                self.patchstar_list[max_moved_idx].x,
                                                                                self.patchstar_list[max_moved_idx].y,
                                                                                self.patchstar_list[max_moved_idx].z,
                                                                                self.patchstar_list[max_moved_idx].a))

        for idx in range(self.patchstar_num):
            if not self.patchstar_list_coordinates[idx]:
                #Move probe away in the max x and z directions
                self.patchstar_list[idx].moveAbsolute(  self.patchstar_list[idx].x_max,
                                                        self.patchstar_list[idx].y_origin,
                                                        self.patchstar_list[idx].z_max)
            else:
                #Move probe away in the max x and z directions
                self.patchstar_list[idx].moveAbsolute(  self.patchstar_list[idx].x_max,
                                                        self.patchstar_list_coordinates[idx][1],
                                                        self.patchstar_list[idx].z_max)
        end_live_update(self)
        config_lu = LiveUpdateConfig()
        config_lu.sls_flag = True
        config_lu.pts_flags = [True]*self.patchstar_num
        config_lu.show_result = False
        start_live_update(self, config_lu)
        return True, []

    def pst_probe_adjustment(self):
        for idx in range(self.patchstar_num):
            if not self.patchstar_list_coordinates[idx]:
                return False, "PatchStar-{} Position Adjustment not Successful.".format(idx)
        return True, []

    def calibrate(self):

        self.cam.get_frame()
        width = self.cam.size[1]
        height = self.cam.size[0]
        #dim = (width, height)

        #Move each patchstar back into camera FOV and find the tip

        for idx in range(self.patchstar_num):
            self.patchstar_curr_idx = idx
            input('Press ENTER')

            self.slicescope.moveAbsolute(self.patchstar_list_slicescope_coordinates[idx][0],self.patchstar_list_slicescope_coordinates[idx][1],self.patchstar_list_slicescope_coordinates[idx][2])
            #self.patchstar_list[idx].approachAbsolute(self.patchstar_list_coordinates[idx][3])
            self.patchstar_list[idx].moveAbsolute(self.patchstar_list_coordinates[idx][0], self.patchstar_list_coordinates[idx][1],self.patchstar_list_coordinates[idx][2])

            self.low_mag_cal()

            self.medium_level_cal()

            self.high_mag_cal()
            #self.high_mag_calibration()

            #self.condenser.moveAbsolute(self.condenser.z_max)

            #Move probe away in the max x and z directions
            #self.patchstar_list[idx].moveAbsolute(self.patchstar_list[idx].x_max, self.patchstar_list[idx].y_origin,self.patchstar_list[idx].z_max)
            #self.patchstar_list[idx].approachAbsolute(self.patchstar_list[idx].a_max)
            self.patchstar_list[idx].approachRelative(self.patchstar_list[idx].a, 1200_00)

            #Update the coordinates for each patchstar with tip in focus and slicescope
            self.slicescope.coordinates()
            self.patchstar_list_slicescope_coordinates[idx] = [self.slicescope.x, self.slicescope.y, self.slicescope.z]

            self.patchstar_list[idx].coordinates()  #Update each patchstar x,y,z,a coordinates

            self.patchstar_list[idx].x_tip = self.patchstar_list[idx].x
            self.patchstar_list[idx].y_tip = self.patchstar_list[idx].y
            self.patchstar_list[idx].z_tip = self.patchstar_list[idx].z
            self.patchstar_list[idx].a_tip = self.patchstar_list[idx].a


            #Move probe away in the max x and z directions
            #self.patchstar_list[idx].moveAbsolute(self.patchstar_list[idx].x_max, self.patchstar_list[idx].y_origin,self.patchstar_list[idx].z_max)
            #self.patchstar_list[idx].approachRelative(self.patchstar_list[idx].a, 1200_00)

        end_live_update(self)
        cv.destroyAllWindows()

        #Move condenser back to original position
        #self.condenser.moveAbsolute(self.condenser.z_origin)

    def low_mag_cal(self):

        end_live_update(self)
        self.cam.set_exp_time(self.low_mag_cal_exp_time)

        for idx in range(self.patchstar_num):
            self.patchstar_curr_idx = idx
            print("Start Patchstar {} Calibration".format(idx))

            # Move Slicescope to pre-set location
            self.slicescope.moveAbsolute(   self.patchstar_list_slicescope_coordinates[idx][0],
                                            self.patchstar_list_slicescope_coordinates[idx][1],
                                            self.patchstar_list_slicescope_coordinates[idx][2])

            # Move Patchstar to User selected location
            self.patchstar_list[idx].moveAbsolute(  self.patchstar_list_coordinates[idx][0],
                                                    self.patchstar_list_coordinates[idx][1],
                                                    self.patchstar_list_coordinates[idx][2])
            #Find tip algorithms
            probe_Z_search(self, show_result=False)


            if not coarse_tip_XY_search(self, show_result=False):
                input("!!!!!!!!!! Coarse tip XY search failed. Check what happened! !!!!!!!!!!")

            print('---------- Coarse tip XY search complete. ----------')

            if not fine_3d_tip_search(self, show_result=False):
                input("!!!!!!!!!! Fine 3d tip search failed. Check what happened! !!!!!!!!!!")

            print('---------- Fine 3d tip search complete. ----------')

            coarse_pts2cam(self, Point(self.patchstar_list[idx].tip_coord_xy[-1]))
            print('---------- Coarse patchstar -> camera coordinate transform complete. ----------')

            fine_pts2cam(self)
            print('---------- Fine patchstar -> camera coordinate transform complete. ----------')

            tip_z_found = pts2slsZ(self, show_result=False)
            print('---------- Patchstar -> Slicescope Z coordinate transform complete. ----------')
            input("---------- Tip Found: {}?".format(tip_z_found))

            self.patchstar_list[idx].moveAbsolute(  self.patchstar_list[idx].x_max,
                                                    self.patchstar_list_coordinates[idx][1],
                                                    self.patchstar_list[idx].z_max)

    def medium_level_cal(self, pts_index = 0):

        end_live_update(self)

        self.patchstar_curr_idx = pts_index
        
        #Move patchstar{idx} to high mag center on low mag (x,y), low_mag_water_z_coord. Use pts2slsZ transformation to calculate low_mag_water_coord for each patchstar.
        move_tip_2_high_mag_centre_on_low_mag(self, show_result=False)

        touch_water = low_mag_touch_water(self, show_result=False)

        if touch_water:

            #self.slicescope.coordinates()
            #self.patchstar_list[idx].coordinates()
            #sls_touch_water_coord = [self.slicescope.x,self.slicescope.y,self.slicescope.low_mag_water_z_coord]
            #pts_touch_water_coord = [self.patchstar_list[idx].x,self.patchstar_list[idx].y,self.patchstar_list[idx].low_mag_water_z_coord]

            ##Use for reference
            ##sp.patchstar_list[curr_idx].pts2slsZ = [x_vec_pts2slsZ[0][0],x_vec_pts2slsZ[1][0]]
            ##sp.patchstar_list[curr_idx].sls2ptsZ = [x_vec_sls2ptsZ[0][0],x_vec_sls2ptsZ[1][0]]
            #sp.slicescope.low_mag_water_z_coord = int(sp.patchstar_list[curr_idx].low_mag_water_z_coord * sp.patchstar_list[curr_idx].pts2slsZ[0] + sp.patchstar_list[curr_idx].pts2slsZ[1])

            for idx in range(self.patchstar_num):
                self.patchstar_list[idx].low_mag_water_z_coord = int(self.slicescope.low_mag_water_z_coord*self.patchstar_list[idx].sls2ptsZ[0] + self.patchstar_list[idx].sls2ptsZ[1])

                print(f'patchstar{idx} low mag water z coord = {self.patchstar_list[idx].low_mag_water_z_coord}')

            #Move probe away in the max x and z directions
            self.patchstar_list[pts_index].moveAbsolute(  self.patchstar_list[pts_index].x_max,
                                                self.patchstar_list_coordinates[pts_index][1],
                                                self.patchstar_list[pts_index].z_max)

            return

        else:
            print(f'Patchstar{pts_index} tip did not touch water. Did not do High Magnification probe/tip search. Restart procedure!!!')

            #Move probe away in the max x and z directions
            self.patchstar_list[idx].moveAbsolute(  self.patchstar_list[pts_index].x_max,
                                                    self.patchstar_list_coordinates[pts_index][1],
                                                    self.patchstar_list[pts_index].z_max)

            return

            #Move condenser back to original position
            self.condenser.moveAbsolute(self.condenser.z_origin)

            while True:

                #High Mag fine tip search Z and XY

                answer1 = input('---------- Put sample in chamber? ').lower()

                if not answer1:
                    continue
                elif answer1.startswith('n'):

                    while True:
                        answer2 = input('---------- Switch to High Magnification? ').lower()

                        if not answer2:
                            continue
                        elif answer2.startswith('n'):
                            break
                        elif answer2.startswith('y'):
                            input('---------- Press [enter] when High Magnification is ready')
                            break
                        else:
                            print('Enter yes or no.')

                elif answer1.startswith('y'):

                    #Move probe away in the max a direction
                    self.patchstar_list[idx].approachAbsolute(self.patchstar_list[idx].a_max)

                    config_lu = LiveUpdateConfig()
                    config_lu.sls_flag = False
                    config_lu.pts_flags =  [False]*self.patchstar_num
                    config_lu.cross = self.high_mag_centre_on_low_mag
                    start_live_update(self, config_lu)

                    while True:
                        answer3 = input('Is the sample at the correct location? ')

                        if not answer3:
                            continue
                        elif answer3.startswith('n'):
                            print(' Fix it!!!')
                            continue
                        elif answer3.startswith('y'):

                            self.slicescope.moveAbsolute(sls_touch_water_coord[0],sls_touch_water_coord[1],sls_touch_water_coord[2])
                            self.patchstar_list[idx].moveAbsolute(pts_touch_water_coord[0],pts_touch_water_coord[1],pts_touch_water_coord[2])

                            input('Press [enter] to continue.')

                            end_live_update(self)

                            break
                        else:
                            print('Enter yes or no.')

                else:
                    print('Enter yes or no.')


    def place_sample_switch_high_mag(self):
        end_live_update(self)

        self.condenser.moveAbsolute(self.condenser.z_origin)

        config_lu = LiveUpdateConfig()
        config_lu.sls_flag = False
        config_lu.pts_flags =  [False]*self.patchstar_num
        config_lu.cross = self.high_mag_centre_on_low_mag
        start_live_update(self, config_lu)
        while True:
            if not input('---------- Put sample in chamber and Press [Enter] '):
                break
        while True:
            if not input('---------- Adjust sample position and Press [Enter] '):
                break
        end_live_update(self)

        while True:
            if not input('---------- Switch to high magnification and Press [Enter] '):
                break

        sls_into_water_max_delta = self.low_mag_focal_plane_2_high_mag_surf + self.low_mag_focal_plane_2_high_mag_surf_margin

        sls_z_move_to = self.slicescope.low_mag_water_z_coord + int(sls_into_water_max_delta)

        self.slicescope.moveAbsolute(self.slicescope.x,self.slicescope.y,sls_z_move_to)

        config_lu = LiveUpdateConfig()
        config_lu.cross = True
        start_live_update(self, config_lu)
        input("Only move the slicescope in the Z axis to focus on the cell. Press [enter] to continue.")
        end_live_update(self)

        self.slicescope.coordinates()
        self.slicescope.cell_z_coord = self.slicescope.z


    def high_mag_cal(self, pts_index=0):

        end_live_update(self)
        self.cam.set_exp_time(self.high_mag_cal_exp_time)

        self.patchstar_curr_idx = pts_index

        config_lu = LiveUpdateConfig()
        config_lu.cross = True
        start_live_update(self, config_lu)
        #start_live_update(self, cross=True)
        self.adjust_intensity()
        end_live_update(self)
        
        step = 150_00
        #self.patchstar_list[idx].moveAbsolute(self.patchstar_list[idx].x, self.patchstar_list[idx].y,self.patchstar_list[idx].z - int(step/2))
        #coord_img = pts_creepZ_image_coord(self, step, abs(int(step/10)))
        #self.img_proc.write_image_stack_to_file(coord_img['norm_img'], ['image_data', 'fine_tip_z_search'])

        high_mag_probe_search(self, step, abs(int(step/10)), show_result=False)

        config_lu = LiveUpdateConfig()
        start_live_update(self, config_lu)
        #start_live_update(self)
        input("Press [enter] to start High Mag Cal - Z coordinate")
        end_live_update(self)

        focal_plane_to_cell_distance = int(abs((self.slicescope.cell_z_coord + self.slicescope.cell_z_coord_margin) - self.slicescope.z))
        print(f'Distance to cell + margin = {focal_plane_to_cell_distance}')

        high_mag_tip_found = pts2slsZ_high_mag(self,focal_plane_to_cell_distance)

        config_lu = LiveUpdateConfig()
        start_live_update(self, config_lu)
        print('pts2slsZ high mag calibrated.')
        input("Press [enter] to start High Mag Cal - X,Y coordinates")
        end_live_update(self)

        if high_mag_tip_found == True:

            #High Mag coarse calibration with 2 points

            high_mag_coarse_pts2cam(self, Point(self.patchstar_list[pts_index].tip_coord_xy[-1]))
            print('---------- High Mag - Coarse patchstar -> camera coordinate transform complete. ----------')

            #High Mag fine calibration with 5 points

            high_mag_fine_pts2cam_2d(self)
            print('---------- High Mag - Fine patchstar -> camera coordinate transform complete. ----------')

            #Print coordinate transforms to screen
            print(f'Low  Mag pts2cam = \n {self.patchstar_list[pts_index].pts2cam["tran_xy"]}')
            print(f'High Mag pts2cam = \n {self.patchstar_list[pts_index].pts2cam_high["tran_xy"]}')

            print(f'Low  Mag cam2pts = \n {self.patchstar_list[pts_index].cam2pts["tran_xy"]}')
            print(f'High Mag cam2pts = \n {self.patchstar_list[pts_index].cam2pts_high["tran_xy"]}')

            config_lu.sls_flag = False
            config_lu.pts_flags = [False]*self.patchstar_num
            config_lu.cross = True
            start_live_update(self, config_lu)
            print('pts2cam high mag calibrated.')
            input("Press [enter]")

        else:
            print('Probe tip not found in High Mag Calibration!!!')
            return

    def high_mag_calibration(self, idx):

        move_high_mag_into_water(self)

        self.patchstar_list[idx].moveAbsolute(self.patchstar_list[idx].x, self.patchstar_list[idx].y,self.patchstar_list[idx].z + self.low_mag_2_high_mag_focal_plane)   #add the difference between low mag and high mag focal plane
        
        config_lu = LiveUpdateConfig()
        config_lu.cross = True
        start_live_update(self, config_lu)
        self.adjust_intensity()
        end_live_update(self)

        step = 150_00
        #self.patchstar_list[idx].moveAbsolute(self.patchstar_list[idx].x, self.patchstar_list[idx].y,self.patchstar_list[idx].z - int(step/2))
        #coord_img = pts_creepZ_image_coord(self, step, abs(int(step/10)))
        #self.img_proc.write_image_stack_to_file(coord_img['norm_img'], ['image_data', 'fine_tip_z_search'])

        high_mag_probe_search(self, step, abs(int(step/10)), show_result=True)

        config_lu = LiveUpdateConfig()
        start_live_update(self, config_lu)
        #start_live_update(self)
        input("Press any key and [Enter] on Terminal to End Live Update")
        end_live_update(self)

        #High Mag coarse calibration and first 4 points. Need to add more points in high_mag_fine_pts2cam and move slicescope relative down Z coord.

        high_mag_coarse_pts2cam(self, Point(self.patchstar_list[idx].tip_coord_xy[-1]))
        print('---------- High Mag - Coarse patchstar -> camera coordinate transform complete. ----------')

        high_mag_fine_pts2cam_3d(self)
        print('---------- High Mag - Fine patchstar -> camera coordinate transform complete. ----------')

        print(f'Low  Mag pts2cam = \n {self.patchstar_list[idx].pts2cam["tran_xy"]}')
        print(f'High Mag pts2cam = \n {self.patchstar_list[idx].pts2cam_high["tran_xy"]}')

        print(f'Low  Mag cam2pts = \n {self.patchstar_list[idx].cam2pts["tran_xy"]}')
        print(f'High Mag cam2pts = \n {self.patchstar_list[idx].cam2pts_high["tran_xy"]}')

        config_lu.sls_flag = False
        config_lu.pts_flags = [False]*self.patchstar_num
        config_lu.cross = True
        start_live_update(self, config_lu)


    def manually_select_target_cells(self, pts_index = 0):
        end_live_update(self)

        #Prompt user to place sample into chamber, switch to High Magnification, and select target cells to patch
        slct_coord = [int(self.cam.size[1]/2), int(self.cam.size[0]/2)]

        config_lu = LiveUpdateConfig()
        config_lu.cross = False
        start_live_update(self, config_lu)
        #start_live_update(self, cross=slct_coord)
        while True:
            #Put cross at proposed high magnification center x,y coordinate
            # Press ESC to close.
            
            #self.cam.video_cross(slct_coord[0], slct_coord[1])
            str = input("Previous cross coordinate: H{}, V{}, New coordinate? ".format(slct_coord[0], slct_coord[1]))
            if not str:
                end_live_update(self)
                config_lu.sls_flag = False
                config_lu.pts_flags = [False]*self.patchstar_num
                config_lu.cross = slct_coord
                start_live_update(self, config_lu)
                #start_live_update(self, sls_flag=False, pts_flags=False, cross=slct_coord)
                break
            slct_coord = [eval(i) for i in str.split(',')]
            end_live_update(self)

            config_lu = LiveUpdateConfig()
            config_lu.cross = slct_coord
            start_live_update(self, config_lu)
            #start_live_update(self, cross=slct_coord)
        
        self.slicescope.coordinates()
        cam_xy_sls_z = Point([slct_coord[0], slct_coord[1], self.slicescope.z])
        pts_to_move = cam_xy_sls_z.trans_3d(self.patchstar_list[pts_index].cam2pts_high_3d["tran_xyz"], self.patchstar_list[pts_index].cam2pts_high_3d["org"])
        self.target_list.append(pts_to_move)

        return slct_coord

    def select_target_cells(self, screen_coords):
        end_live_update(self)
        
        self.slicescope.coordinates()
        config_lu = LiveUpdateConfig()
        config_lu.sls_flag = False
        config_lu.pts_flags = [False]*self.patchstar_num
        config_lu.cross = screen_coords
        start_live_update(self, config_lu)

        cam_xy_sls_z = Point([screen_coords[0], screen_coords[1], self.slicescope.z])
        pts_to_move = cam_xy_sls_z.trans_3d(self.patchstar_list[pts_index].cam2pts_high_3d["tran_xyz"], self.patchstar_list[pts_index].cam2pts_high_3d["org"])
        self.target_list.append(pts_to_move)

    def guess_screen_coord(self):

        while True:

            screen_coord = list(map(int, input("Type in x y coordinates to put cross on the screen: ").strip().split()))[:2]

            frame = self.cam.get_frame()

            self.img_proc.cross(frame, screen_coord[0],screen_coord[1])

            if not screen_coord:
                break

            cv.imshow('Guess Screen Coordinates',frame)
            cv.waitKey(0)

    def pts_trajectory_angle(self, pts_index = 0):

        end_live_update(self)

        self.patchstar_curr_idx = pts_index

        #Get point 1 of patchstar approach trajectory
        self.patchstar_list[pts_index].coordinates()
        point1 = [self.patchstar_list[pts_index].x,self.patchstar_list[pts_index].y,self.patchstar_list[pts_index].z,self.patchstar_list[pts_index].a]
        print(point1)

        #Move probe out to get point 2 of approach trajectory
        self.patchstar_list[pts_index].approachRelative(self.patchstar_list[pts_index].a,1000_00)

        self.patchstar_list[pts_index].coordinates()
        point2 = [self.patchstar_list[pts_index].x,self.patchstar_list[pts_index].y,self.patchstar_list[pts_index].z,self.patchstar_list[pts_index].a]
        print(point2)

        #Calculate probe trajectory angle

        dz = abs(point2[2] - point1[2])
        dx = abs(point2[0] - point1[0])

        print(f'dz = {dz}')
        print(f'dx = {dx}')

        angle = np.arctan2(dz,dx) * 180/np.pi


        #Cross: (sp.high_mag_centre_on_low_mag[0],sp.high_mag_centre_on_low_mag[1])
        high_mag_centre_on_low_mag_coord = Point(self.high_mag_centre_on_low_mag)
        pts_to_move_xy = high_mag_centre_on_low_mag_coord.trans_2d(self.patchstar_list[pts_index].cam2pts["tran_xy"],
                                                                    self.patchstar_list[pts_index].cam2pts["org"])

        #Get x_stop,z_stop of patchstar approach trajectory
        x_stop = pts_to_move_xy.x
        z_stop = self.patchstar_list[pts_index].low_mag_water_z_coord

        while angle > 42.5:

            step = -1000_00
            self.patchstar_list[pts_index].moveRelative(self.patchstar_list[pts_index].x,self.patchstar_list[pts_index].y,self.patchstar_list[pts_index].z,0,0,step)

            #Get x_start,z_start of patchstar approach trajectory
            x_start = self.patchstar_list[pts_index].x
            z_start = self.patchstar_list[pts_index].z

            #Calculate probe trajectory angle

            dz = abs(z_start - z_stop)
            dx = abs(x_start - x_stop)

            print(f'dz = {dz}')
            print(f'dx = {dx}')

            angle = np.arctan2(dz,dx) * 180/np.pi

        self.patchstar_list[pts_index].probe_angle = angle
        print(f'Probe{pts_index} angle = {self.patchstar_list[pts_index].probe_angle}')


    def patch(self, pts_index = 0):
        if self.target_list:
            pnt_target = self.target_list.pop(0)
            input(f'The target is {pnt_target.col}?')
            self.patchstar_list[pts_index].coordinates()
            self.patchstar_list[pts_index].moveAbsolute(pnt_target.x, pnt_target.y,pnt_target.z)


    def test_background_probe_histograms(self):

        end_live_update(self)

        #Move each patchstar back into camera FOV and find the tip

        for idx in range(self.patchstar_num):
            self.patchstar_curr_idx = idx

            while True:

                config_lu = LiveUpdateConfig()
                start_live_update(self, config_lu)
                #start_live_update(self)
                input('Clear camera FOV. Adjust lighting and iris.')
                end_live_update(self)

                background_img = self.cam.get_raw_frame(10)
                background_img_hist, background_idx_hist =  np.histogram(background_img.flatten(), bins=100)

                start_live_update(self, config_lu)
                #start_live_update(self)
                input('Move probe into camera FOV.')
                end_live_update(self)

                #Save coordinates for each probe
                #Update the slicescope x,y,z coordinates for each patchstar
                self.slicescope.coordinates()
                self.patchstar_list_slicescope_coordinates[idx] = [self.slicescope.x, self.slicescope.y, self.slicescope.z]

                #Update each patchstar x,y,z,a coordinates
                self.patchstar_list[idx].coordinates()
                self.patchstar_list_coordinates[idx] = [self.patchstar_list[idx].x, self.patchstar_list[idx].y, self.patchstar_list[idx].z, self.patchstar_list[idx].a]
                print("User Moves PatchStar{} to Coordinates: {}, {}, {}, {}".format(idx,   self.patchstar_list[idx].x,
                                                                                            self.patchstar_list[idx].y,
                                                                                            self.patchstar_list[idx].z,
                                                                                            self.patchstar_list[idx].a))

                #Find tip algorithms
                probe_Z_search(self, show_result=False)

                self.cam.get_frame()
                cv.imshow('Probe body in focus',self.cam.frame_norm)
                cv.waitKey(5)

                probe_img = self.cam.get_raw_frame(10)
                probe_img_hist, probe_idx_hist = np.histogram(probe_img.flatten(), bins=100)

                #bins=np.histogram(np.hstack((self.patchstar_list[idx].background_img_hist,self.patchstar_list[idx].probe_img_hist)), bins=100)[1] #get the bin edges
                #plt.hist(self.patchstar_list[idx].background_img_hist, bins)
                #plt.hist(self.patchstar_list[idx].probe_img_hist, bins)
                plt.plot(background_idx_hist[1:], background_img_hist)
                plt.plot(probe_idx_hist[1:], probe_img_hist)
                plt.grid()
                plt.show()

                answer = input('Quit?').lower()

                if not answer:
                    continue
                elif answer.startswith('n'):
                    continue
                elif answer.startswith('y'):
                    break
                else:
                    print('Enter yes or no.')

            return

    def manually_select_coord(self, cam_xy_sls_z_stack = PointStack([])):
        end_live_update(self)

        #Prompt user to place sample into chamber, switch to High Magnification, and select target cells to patch
        if cam_xy_sls_z_stack.np == 0:
            slct_coord = [int(self.cam.size[1]/2), int(self.cam.size[0]/2)]
        else:
            slct_coord = [cam_xy_sls_z_stack.index(cam_xy_sls_z_stack.np - 1).col[0, 0], cam_xy_sls_z_stack.index(cam_xy_sls_z_stack.np - 1).col[1, 0]]

        config_lu = LiveUpdateConfig()
        config_lu.sls_flag = False
        config_lu.pts_flags = [False]*self.patchstar_num
        config_lu.cross = slct_coord
        config_lu.b_mouse_callback = True
        start_live_update(self, config_lu)
        #start_live_update(self, sls_flag=False, pts_flags=False, cross=slct_coord, b_mouse_callback=True)
        input("Has the screen mouse click happened? ")
        self.slicescope.coordinates()
        cam_xy_sls_z_stack.append(Point([self.mouse_click_coord["x"], self.mouse_click_coord["y"], self.slicescope.z]))
        print("Input CAM/SLS coordinate: {}, {}, {} ".format(self.mouse_click_coord["x"], self.mouse_click_coord["y"], self.slicescope.z))

    def test_pts_approach(self,  pts_index = 0):

        end_live_update(self)

        #self.patchstar_list[pts_index].cam2pts_high_3d = {"tran_xyz":0, "org":0}
        #self.patchstar_list[pts_index].cam2pts_high_3d["tran_xyz"] = np.array([ [-1.55239575e+01, 5.72404999e+00, -1.11663925e-02],
        #                                                                        [ 5.68776829e+00, 1.51706550e+01,  4.82403027e-03],
        #                                                                        [-8.38265294e-02, 7.04789560e-02,  1.00794138e+00]])

        #self.patchstar_list[pts_index].cam2pts_high_3d["org"] = Point([50042.72724019, 93908.32495937, -50681.48172489])

        self.patchstar_list[pts_index].coordinates()
        pts_withdraw_coordinate = [self.patchstar_list[pts_index].x, self.patchstar_list[pts_index].y, self.patchstar_list[pts_index].z]
        input("Press any key and change to Low mag, and place the sample")

        config_lu = LiveUpdateConfig()
        config_lu.sls_flag = False
        config_lu.pts_flags = [False]*self.patchstar_num
        config_lu.cross = self.high_mag_centre_on_low_mag
        start_live_update(self, config_lu)
        #start_live_update(self, sls_flag=False, pts_flags=False, cross=self.high_mag_centre_on_low_mag)
        input("Press any key to after placed the sample and changed back to high mag")
        self.patchstar_list[pts_index].moveAbsolute(pts_withdraw_coordinate[0], pts_withdraw_coordinate[1], pts_withdraw_coordinate[2])
        self.slicescope.coordinates()
        self.slicescope.creepX(self.patchstar_list[pts_index].cam2pts_high_3d["sls_coord"].x - self.slicescope.x, 20_00)
        self.slicescope.creepY(self.patchstar_list[pts_index].cam2pts_high_3d["sls_coord"].y - self.slicescope.y, 20_00)
        self.slicescope.creepZ(self.patchstar_list[pts_index].cam2pts_high_3d["sls_coord"].z - self.slicescope.z, 20_00)
        self.slicescope.wait()

        self.slicescope.set_top_speed(20)
        self.slicescope.check_top_speed()

        self.patchstar_list[pts_index].set_top_speed(20)
        while True:
            cam_xy_sls_z_input_stack = PointStack([])
            cam_xy_sls_z_target_stack = PointStack([])
            cam_xy_sls_z_result_stack = PointStack([])
            n_man_calib = 3
            for idx in range(n_man_calib):
                input("Set Calibration Target.")
                self.manually_select_coord(cam_xy_sls_z_stack = cam_xy_sls_z_input_stack)
                cam_xy_sls_z_target_stack.append(cam_xy_sls_z_input_stack.index(cam_xy_sls_z_input_stack.np - 1))
                pts_to_move = cam_xy_sls_z_target_stack.index(cam_xy_sls_z_target_stack.np - 1).trans_3d(self.patchstar_list[pts_index].cam2pts_high_3d["tran_xyz"],
                                                                                                        self.patchstar_list[pts_index].cam2pts_high_3d["org"])
                self.target_list.append(pts_to_move)
                self.patch()
                input("Move cross to Tip Location.")
                self.manually_select_coord(cam_xy_sls_z_stack = cam_xy_sls_z_input_stack)
                cam_xy_sls_z_result_stack.append(cam_xy_sls_z_input_stack.index(cam_xy_sls_z_input_stack.np - 1))

                self.adjust_intensity()
                start_live_update(self, config_lu)
                input("Press any key for next round.")

                self.patchstar_list[pts_index].coordinates()
                self.patchstar_list[pts_index].approachRelative(self.patchstar_list[pts_index].a, 1200_00)

            err_corr_pnt = Point(np.mean(cam_xy_sls_z_target_stack.cols - cam_xy_sls_z_result_stack.cols, axis=1))
            print("Error Correction Vector is :\n{}".format(err_corr_pnt.col))
            cam_xy_sls_z_input_stack = PointStack([])
            while True:
                input("Set patch Target.")
                self.manually_select_coord(cam_xy_sls_z_stack = cam_xy_sls_z_input_stack)
                cam_xy_sls_z_target_pnt = cam_xy_sls_z_input_stack.index(cam_xy_sls_z_input_stack.np - 1) + err_corr_pnt
                pts_to_move = cam_xy_sls_z_target_pnt.trans_3d(self.patchstar_list[pts_index].cam2pts_high_3d["tran_xyz"],
                                                            self.patchstar_list[pts_index].cam2pts_high_3d["org"])
                self.target_list.append(pts_to_move)
                self.patch()
                str = input("Press any key for next round, press [Enter] only to re-do error correction.")
                if not str:
                    self.patchstar_list[pts_index].coordinates()
                    self.patchstar_list[pts_index].approachRelative(self.patchstar_list[pts_index].a, 1200_00)
                    break

                self.patchstar_list[pts_index].coordinates()
                self.patchstar_list[pts_index].approachRelative(self.patchstar_list[pts_index].a, 1200_00)


    def error_correct_pts_approach(self, pts_index = 0):

        end_live_update(self)

        self.patchstar_curr_idx = pts_index

        #patchstar should already be moved out of camera FOV from trajectory approach function. This last location is the withdraw or 'starting' coordinate
        self.patchstar_list[pts_index].coordinates()
        pts_withdraw_coordinate = [self.patchstar_list[pts_index].x, self.patchstar_list[pts_index].y, self.patchstar_list[pts_index].z]

        config_lu = LiveUpdateConfig()
        config_lu.sls_flag = False
        config_lu.pts_flags = [False]*self.patchstar_num
        config_lu.cross = self.high_mag_centre_on_low_mag
        start_live_update(self, config_lu)
        input("Press [enter] to start error correction for tip approach")

        self.patchstar_list[pts_index].moveAbsolute(pts_withdraw_coordinate[0], pts_withdraw_coordinate[1], pts_withdraw_coordinate[2])
        self.slicescope.coordinates()

        self.slicescope.creepX(self.patchstar_list[pts_index].cam2pts_high["sls_coord"].x - self.slicescope.x, 20_00)
        self.slicescope.creepY(self.patchstar_list[pts_index].cam2pts_high["sls_coord"].y - self.slicescope.y, 20_00)
        self.slicescope.creepZ(self.patchstar_list[pts_index].cam2pts_high["sls_coord"].z - self.slicescope.z, 20_00)
        self.slicescope.wait()

        #self.slicescope.set_top_speed(20)
        #self.slicescope.check_top_speed()

        self.patchstar_list[pts_index].set_top_speed(20)
        while True:
            cam_xy_sls_z_input_stack = PointStack([])
            cam_xy_sls_z_target_stack = PointStack([])
            cam_xy_sls_z_result_stack = PointStack([])
            n_man_calib = 3
            for num in range(n_man_calib):
                input("Set Calibration Target. Move slicescope only in Z direction to focus.")
                self.manually_select_coord(cam_xy_sls_z_stack = cam_xy_sls_z_input_stack)
                cam_xy_sls_z_target_stack.append(cam_xy_sls_z_input_stack.index(cam_xy_sls_z_input_stack.np - 1))
                pts_to_move_xy = cam_xy_sls_z_target_stack.index(cam_xy_sls_z_target_stack.np - 1).trans_2d(self.patchstar_list[pts_index].cam2pts_high["tran_xy"],
                                                                                                        self.patchstar_list[pts_index].cam2pts_high["org"])
                self.slicescope.coordinates()
                pts_to_move_z = Point([0,0,self.slicescope.z * self.patchstar_list[pts_index].sls2ptsZ_high[0] + self.patchstar_list[pts_index].sls2ptsZ_high[1]])
                pts_to_move = pts_to_move_xy.add(pts_to_move_z)

                self.target_list.append(pts_to_move)
                self.patch(pts_index)
                input("Move cross to Tip Location.")
                self.manually_select_coord(cam_xy_sls_z_stack = cam_xy_sls_z_input_stack)
                cam_xy_sls_z_result_stack.append(cam_xy_sls_z_input_stack.index(cam_xy_sls_z_input_stack.np - 1))

                self.adjust_intensity()
                start_live_update(self, config_lu)
                input("Press any key for next round.")

                self.patchstar_list[pts_index].coordinates()
                self.patchstar_list[pts_index].approachRelative(self.patchstar_list[pts_index].a, 1200_00)

            err_corr_pnt = Point(np.mean(cam_xy_sls_z_target_stack.cols - cam_xy_sls_z_result_stack.cols, axis=1))
            print("Error Correction Vector is :\n{}".format(err_corr_pnt.col))

            cam_xy_sls_z_input_stack = PointStack([])
            while True:
                input("Set patch Target.")
                self.manually_select_coord(cam_xy_sls_z_stack = cam_xy_sls_z_input_stack)
                cam_xy_sls_z_target_pnt = cam_xy_sls_z_input_stack.index(cam_xy_sls_z_input_stack.np - 1) + err_corr_pnt
                pts_to_move_xy = cam_xy_sls_z_target_pnt.trans_2d(self.patchstar_list[pts_index].cam2pts_high["tran_xy"],
                                                                                                        self.patchstar_list[pts_index].cam2pts_high["org"])
                self.slicescope.coordinates()
                pts_to_move_z = Point([0,0,self.slicescope.z * self.patchstar_list[pts_index].sls2ptsZ_high[0] + self.patchstar_list[pts_index].sls2ptsZ_high[1]])
                pts_to_move = pts_to_move_xy.add(pts_to_move_z)

                self.target_list.append(pts_to_move)
                self.patch(pts_index)
                str = input("Press any key for next round, press [Enter] only to re-do error correction.")
                if not str:
                    self.patchstar_list[pts_index].coordinates()
                    self.patchstar_list[pts_index].approachRelative(self.patchstar_list[pts_index].a, 1200_00)
                    break

                self.patchstar_list[pts_index].coordinates()
                self.patchstar_list[pts_index].approachRelative(self.patchstar_list[pts_index].a, 5000_00)


    def pts_target_patch(self, pts_index = 0):

        end_live_update(self)

        self.patchstar_curr_idx = pts_index

        #patchstar should already be moved out of camera FOV from trajectory approach function. This last location is the withdraw or 'starting' coordinate
        self.patchstar_list[pts_index].coordinates()
        pts_withdraw_coordinate = [self.patchstar_list[pts_index].x, self.patchstar_list[pts_index].y, self.patchstar_list[pts_index].z]

        config_lu = LiveUpdateConfig()
        config_lu.sls_flag = False
        config_lu.pts_flags = [False]*self.patchstar_num
        config_lu.cross = self.high_mag_centre_on_low_mag
        start_live_update(self, config_lu)
        input("Press [enter] to start patching.")

        self.patchstar_list[pts_index].moveAbsolute(pts_withdraw_coordinate[0], pts_withdraw_coordinate[1], pts_withdraw_coordinate[2])
        self.slicescope.coordinates()

        self.slicescope.creepX(self.patchstar_list[pts_index].cam2pts_high["sls_coord"].x - self.slicescope.x, 20_00)
        self.slicescope.creepY(self.patchstar_list[pts_index].cam2pts_high["sls_coord"].y - self.slicescope.y, 20_00)
        self.slicescope.creepZ(self.patchstar_list[pts_index].cam2pts_high["sls_coord"].z - self.slicescope.z, 20_00)
        self.slicescope.wait()

        #self.slicescope.set_top_speed(20)
        #self.slicescope.check_top_speed()

        self.patchstar_list[pts_index].set_top_speed(20)

        cam_xy_sls_z_input_stack = PointStack([])
        while True:
            input("Set patch Target.")
            self.manually_select_coord(cam_xy_sls_z_stack = cam_xy_sls_z_input_stack)
            cam_xy_sls_z_target_pnt = cam_xy_sls_z_input_stack.index(cam_xy_sls_z_input_stack.np - 1)
            pts_to_move_xy = cam_xy_sls_z_target_pnt.trans_2d(self.patchstar_list[pts_index].cam2pts_high["tran_xy"],
                                                                                                    self.patchstar_list[pts_index].cam2pts_high["org"])
            self.slicescope.coordinates()
            pts_to_move_z = Point([0,0,self.slicescope.z * self.patchstar_list[pts_index].sls2ptsZ_high[0] + self.patchstar_list[pts_index].sls2ptsZ_high[1]])
            pts_to_move = pts_to_move_xy.add(pts_to_move_z)

            self.target_list.append(pts_to_move)
            self.patch(pts_index)
            str = input("Press any key for next round, press [Enter] only to re-do error correction.")
            if not str:
                self.patchstar_list[pts_index].coordinates()
                self.patchstar_list[pts_index].approachRelative(self.patchstar_list[pts_index].a, 1200_00)
                break

            self.patchstar_list[pts_index].coordinates()
            self.patchstar_list[pts_index].approachRelative(self.patchstar_list[pts_index].a, 5000_00)


def main():

    #Object called sp
    sp = SP()
    # SP caller decides which port to use as which instrument
    #for port, desc, hwid in sp.list_port_info:
    #    print(f"{port}: {desc} [{hwid}]")
    #Initialize all of the equipment
    sp.initialize(['COM3', 'COM4', 'COM7', 'COM8'])
    #sp.initialize()
    #Initial configuration
    #sp.configure()

    #Move condenser all the way down
    sp.condenser.moveAbsolute(sp.condenser.z_max)

    #User adjusts the camera intensity, aperture, and exposure time for optimum contrast and no saturation
    print('Adjust the camera intensity, aperture, and exposure time for optimum contrast and no saturation.')
    sp.adjust_intensity()
    
    #sp.test_pts_approach()
    #return
    #input()

    #end_live_update(sp)

    #for i in range(5):
    #    realtime = False
    #    coord_img = pts_creepZ_image_coord(sp, -100_00, 10_00, realtime=realtime)
    #    range_start = -1*len(coord_img['img'])
    #    for j in range(range_start+1,0,1):
    #        cv.imshow('GUI Img Stack', sp.gui_img_stack[j])
    #        cv.waitKey(10)
    #    time.sleep(2)
    #    #input()

    #print(len(coord_img['norm_img']))
    #return
    #end_live_update(sp)
    #sp.guess_screen_coord()
    #start_live_update(sp, sls_flag=False, pts_flags=False, cross=True, b_mouse_callback=True)
    #input("Click on window Live")

    #end_live_update(sp)

    #print(sp.mouse_click_coord)

    #while True:
    #    sp.cam.get_frame()
    #    cv.imshow('Frame', sp.cam.frame_norm)
    #    cv.setMouseCallback('Frame',sp.mouse_click_target)
    #    cv.waitKey(0)
    #    print(sp.mouse_click_coord)

    #coord_img = pts_creepZ_image_coord(sp, 1000_00, 1000_00)
    #sp.img_proc.write_image_stack_to_file(coord_img['img'],'C:/Users/Lab/OneDrive - BioVision Group Limited/Desktop/Test')   #coord_img['img'] coord_img['img_cross'] coord_img['norm_img']
    #return

    #sp.patchstar_trajectory_angle()
    #return

    #end_live_update(sp)
    #sp.cam.video_cross(sp.high_mag_centre_on_low_mag[0],sp.high_mag_centre_on_low_mag[1])

    #sp.cam.get_frame()
    #width = sp.cam.size[1]
    #height = sp.cam.size[0]
    #sp.cam.video_cross(int(width/2),int(height/2))
    #return

    #input('Press ENTER to exit')
    #return

    #sp.test_background_probe_histograms()
    #return

    #Check top speed
    #sp.check_top_speed()

    #Set top speed
    #sp.set_top_speed(4000)  #6000 is ok for slicescope but NOT for patchstars. Better to use 4000 for all equipment.

    #sp.slicescope.moveAbsolute(sp.slicescope.x,sp.slicescope.y, int(sp.slicescope.z_max/3))

    #high_mag_probe_search(sp, 150_00, 15_00, show_result=True)

    sp.probe_adjustment()
    #sp.pseudo_probe_adjustment()

    #while True:
    #    exp_time = int(input("input exp time:"))
    #    if input("press [enter] for normalizing, or any other key for not normalizing"):
    #        b_norm = False
    #    else:
    #        b_norm = True
    #    sp.adjust_intensity(exp_time=exp_time, b_norm=b_norm)

    sp.low_mag_cal()
    sp.medium_level_cal(0)  #default: pts_index = 0

    #test probe coordinate transform to touch water
    #while True:
    #    for idx in range(sp.patchstar_num):

    #        sp.patchstar_curr_idx = idx

    #        #Cross: (sp.high_mag_centre_on_low_mag[0],sp.high_mag_centre_on_low_mag[1])

    #        high_mag_centre_on_low_mag_coord = Point(sp.high_mag_centre_on_low_mag)
    #        pts_to_move_xy = high_mag_centre_on_low_mag_coord.trans_2d(sp.patchstar_list[idx].cam2pts["tran_xy"],
    #                                                                   sp.patchstar_list[idx].cam2pts["org"])

    #        print(pts_to_move_xy.col) #numpy column vector in Point pts_to_move_xy
    #        print(pts_to_move_xy.col[0,0])  #access numpy array element this way. Same as [0][0] from list. Same as pts_to_move_xy.x
    #        print(pts_to_move_xy.col[1,0])  #pts_to_move_xy.y
    #        print(pts_to_move_xy.x)
    #        print(pts_to_move_xy.y)

    #        sp.patchstar_list[idx].moveAbsolute(int(pts_to_move_xy.x), int(pts_to_move_xy.y), sp.patchstar_list[idx].low_mag_water_z_coord)

    #        config_lu = LiveUpdateConfig()
    #        config_lu.sls_flag = False
    #        config_lu.pts_flags = [False]*sp.patchstar_num
    #        config_lu.cross = sp.high_mag_centre_on_low_mag
    #        start_live_update(sp, config_lu)

    #        input('Press enter')

    #        sp.patchstar_list[idx].approachAbsolute(sp.patchstar_list[idx].a_max)

    #        input('Press enter')
    #        end_live_update(sp)

    #    idx=0

    #return

    sp.place_sample_switch_high_mag()

    #test out loop 2 times.
    for num in range(sp.patchstar_num):

        while True:

            pts_index = input(f'Which patchstar to use? Choose 0 - {sp.patchstar_num-1}): ')

            if isinstance(pts_index, int):

                if 0 <= pts_index < sp.patchstar_num:
                    break
                else:
                    print('Please enter a valid patchstar number.')

            else:
                print(f'{pts_index} is not a valid patchstar number. Try again.')

        sp.patchstar_curr_idx = pts_index

        #Move slicescope back up to above water level 
        sp.slicescope.moveAbsolute(sp.slicescope.x,sp.slicescope.y,sp.slicescope.low_mag_water_z_coord)

        #Calculate trajectory angle for chosen patchstar
        sp.pts_trajectory_angle(pts_index)

        #Cross: (sp.high_mag_centre_on_low_mag[0],sp.high_mag_centre_on_low_mag[1])
        high_mag_centre_on_low_mag_coord = Point(sp.high_mag_centre_on_low_mag)
        pts_to_move_xy = high_mag_centre_on_low_mag_coord.trans_2d(sp.patchstar_list[pts_index].cam2pts["tran_xy"],
                                                                    sp.patchstar_list[pts_index].cam2pts["org"])

        sp.patchstar_list[pts_index].moveAbsolute(int(pts_to_move_xy.x), int(pts_to_move_xy.y), sp.patchstar_list[pts_index].low_mag_water_z_coord)

        config_lu = LiveUpdateConfig()
        config_lu.sls_flag = False
        config_lu.pts_flags =  [False]*sp.patchstar_num
        config_lu.cross = sp.high_mag_centre_on_low_mag
        start_live_update(sp, config_lu)
        input('Is the probe tip at the screen target?: Press [enter]')
        end_live_update(sp)

        move_high_mag_into_water(sp, pts_index)

        #Move patchstar up to align from low mag focal plane to high mag focal plane. 
        sp.patchstar_list[pts_index].moveAbsolute(sp.patchstar_list[pts_index].x, sp.patchstar_list[pts_index].y,sp.patchstar_list[pts_index].z + sp.low_mag_2_high_mag_focal_plane)   #add the difference between low mag and high mag focal plane

        sp.high_mag_cal(pts_index)

        #sp.patchstar_trajectory_angle(pts_index)

        sp.pts_target_patch(pts_index)
        #sp.error_correct_pts_approach(pts_index)

        sp.patchstar_list[pts_index].approachAbsolute(sp.patchstar_list[pts_index].a_max)

    return

    #sp.high_mag_calibration(0)
    #input()
    
    #sp.calibrate()
    #return


    #start_live_update(sp)
    #time.sleep(1)
    #end_live_update(sp)

    #sp.test_pts_approach()

    #sp.manually_select_target_cells()
    #sp.patch()


    '''

    sp.select_target_cells()

    #GUI
    root = App(sp.slicescope, sp.condenser, sp.cam, sp.patchstar_list)
    root.mainloop()
    
    sp.live_flag = False
    thread1.join()
    '''

    #Close and disconnect all equipment

    for idx in range(sp.patchstar_num):
        sp.patchstar_list[idx].close()

    sp.cam.disconnect()
    sp.condenser.close()
    sp.slicescope.close()


if __name__ == '__main__':

    main()