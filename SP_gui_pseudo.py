#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tues July 2 12:18:00 2024

@author: Dr. Vincent Lee
Bioptix Ltd.

"""

#Classes
#from serial_port_tools import get_serial_ports

#from slicescope import Slicescope
#from condenser import Condenser
#from patchstar import Patchstar
#from pvcam import PVCAM
#from heka import Heka
#from batch import Batch
#from SP_tkinter import App

#from image_process import Image_process
#from threading import Thread
#from distance_from_focus import *
#from probe_search import *
#from live_update_funcs import *

import time

import numpy as np
import cv2 as cv
import matplotlib.pyplot as plt

class SP:

    def __init__ (self):
        self.gui_img_stack = [] # Live images are pushed to this stack when live_flag, 
                                # Burst of images are pushed to this stack occasionally when live_flag is False
        self.gui_patchstar_coordinates = []     # Latest Coordinate is updated in this
        self.gui_slicescope_coordinates = []    # Latest Coordinate is updated in this

        self.live_flag = False
        self.port_dict = {  'COM4':'Scitifica Slicescope', \
                            'COM5':'Scitifica Condenser', \
                            'COM8':'Scitifica PatchStar', \
                            'COM9':'Scitifica PatchStar'}

    def initialize(self, port_list=['COM4', 'COM5', 'COM8', 'COM9']):
        #   port_list   string list     port_list[0] is the port for slicescope
        #                               port_list[1] is the port for condenser
        #                               port_list[2:end] are ports for patchstars

        start_live_update(self)

    def configure(self, **kwargs):

        end_live_update(self)

        read_configure(**kwargs)

        #Move condenser out of the way

        #Start Slicescope calibration Thread

        #Start PatchStar calibration Thread

        #Move condenser back to original position

    def probe_adjustment(self):
        end_live_update(self)

        #Move condenser away from origin position
        self.condenser.moveRelative(self.condenser.z_origin,15000_00)

        for idx in range(self.patchstar_num):

            start_live_update(self)

            #Prompt user to move probes into camera FOV
            input(f'Manually move the patchstar{idx} into Camera FOV. Press ENTER when finished.')

            end_live_update(self)

            #Save coordinates for each probe
            #Update the slicescope x,y,z coordinates for each patchstar
            self.slicescope.coordinates()
            self.patchstar_list_slicescope_coordinates.append([self.slicescope.x, self.slicescope.y, self.slicescope.z])

            #Update each patchstar x,y,z,a coordinates
            self.patchstar_list[idx].coordinates()
            self.patchstar_list_coordinates[idx] = [self.patchstar_list[idx].x, self.patchstar_list[idx].y, self.patchstar_list[idx].z, self.patchstar_list[idx].a]

            #Move probe away in the max x and z directions
            #self.patchstar_list[idx].approachAbsolute(self.patchstar_list[idx].a_max)
            self.patchstar_list[idx].moveAbsolute(self.patchstar_list[idx].x_max, self.patchstar_list[idx].y_origin,self.patchstar_list[idx].z_max)

        #Prompt user to prepare sample
        input('Press ENTER to continue. Please prepare sample.')

    def calibrate(self):
        #Move each patchstar back into camera FOV and find the tip
        for idx in range(self.patchstar_num):
            self.patchstar_curr_idx = idx
            input('Press ENTER')

            #Find tip algorithms
        #Coordinate Transformation

    def select_target_cells(self):
        #Prompt user to place sample into chamber, switch to High Magnification, and select target cells to patch
        for idx in range(self.patchstar_num):
            
            input(f'Manually move slicescope over target cell{idx}. Press ENTER when finished.')

            #Save coordinates into target list
            self.slicescope.coordinates()
            self.target_list.append([self.slicescope.x, self.slicescope.y, self.slicescope.z])

        #Determine which probe will be optimum to patch target cell. 
        #Perform 3D scan of the cell at these positions to find optimum target location for smart patching trajectory

        for idx in range(self.patchstar_num):
            self.slicescope.moveAbsolute(self.target_list[idx][0],self.target_list[idx][1],self.target_list[idx][2])


    def autofocus(self):
        #High Mag autofocus with probe trajectory and tracking
        pass

    def patch(self):
        pass        


def main():

    #Object called sp
    sp = SP()

    # SP caller decides which port to use as which instrument
    #for port, desc, hwid in sp.list_port_info:
    #    print(f"{port}: {desc} [{hwid}]")
    #Initialize all of the equipment
    sp.initialize()

    #Initial configuration
    sp.configure()


    #Live video and slicescope and patchstar coordinates
    sp.probe_adjustment()

    sp.calibrate()
    #return

    start_live_update(sp)
    time.sleep(5)
    end_live_update(sp)

    #Close and disconnect all equipment

    #for idx in range(sp.patchstar_num):
    #    sp.patchstar_list[idx].close()

    #sp.cam.disconnect()
    #sp.condenser.close()
    #sp.slicescope.close()


if __name__ == '__main__':
    main()