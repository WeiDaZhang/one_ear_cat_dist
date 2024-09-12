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

        self.live_flag = False

        self.target_list = []

    def initialize(self, port_list=['COM8', 'COM9', 'COM4', 'COM5']):
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

        self.cam = PVCAM()
        self.heka = Heka()
        self.batch = Batch()
        start_live_update(self)


    def configure(self):

        end_live_update(self)

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
        #self.condenser.moveRelative(self.condenser.z_origin,18000_00)
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
            start_live_update(self)
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


    def probe_adjustment(self):
        end_live_update(self)

        self.slicescope.moveAbsolute(self.slicescope.x_origin,self.slicescope.y_origin,self.slicescope.z_origin)


        #Move condenser away from origin position
        #self.condenser.moveRelative(self.condenser.z_origin,15000_00)
        self.condenser.moveAbsolute(self.condenser.z_max)

        #Get histogram of baseline background image
        background_img = self.cam.get_raw_frame(10)
        background_img_hist, background_idx_hist = self.img_proc.get_histogram(background_img)

        for idx in range(self.patchstar_num):

            start_live_update(self)

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

            self.patchstar_list[idx].background_img_hist = background_img_hist
            self.patchstar_list[idx].background_idx_hist = background_idx_hist

            #Move probe away in the max x and z directions
            self.patchstar_list[idx].moveAbsolute(self.patchstar_list[idx].x_max, self.patchstar_list[idx].y_origin,self.patchstar_list[idx].z_max)

        #Prompt user to prepare sample
        input('Press ENTER to continue. Please prepare sample.')


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

            #Find tip algorithms
            probe_Z_search(self, show_result=False)

            probe_img = self.cam.get_raw_frame(10)
            probe_img_hist, probe_idx_hist = self.img_proc.get_histogram(probe_img)

            self.patchstar_list[idx].probe_img_hist = probe_img_hist
            self.patchstar_list[idx].probe_idx_hist = probe_idx_hist

            #bins=np.histogram(np.hstack((self.patchstar_list[idx].background_img_hist,self.patchstar_list[idx].probe_img_hist)), bins=100)[1] #get the bin edges
            #plt.hist(self.patchstar_list[idx].background_img_hist, bins)
            #plt.hist(self.patchstar_list[idx].probe_img_hist, bins)
            plt.plot(self.patchstar_list[idx].background_idx_hist[1:], self.patchstar_list[idx].background_img_hist)
            plt.plot(self.patchstar_list[idx].probe_idx_hist[1:], self.patchstar_list[idx].probe_img_hist)
            plt.grid()
            plt.show()

            #cnt_info, cnt_found = probe_thold_contour(self.cam.get_raw_frame(), show_result = True)
            #if cnt_found:
            #    kwargs = {"Found Contour":"Found Contour"}
            #    kw_draw_cnt(self.cam.get_frame(), kwargs, "Found Contour", [cnt_info["contour"]])

            if not coarse_tip_XY_search(self, show_result=False):
                input("Coarse tip XY search failed. Check what happened!")

            if not fine_3d_tip_search(self, show_result=False):
                input("Fine 3d tip search failed. Check what happened!")

            coarse_pts2cam(self, Point(self.patchstar_list[idx].tip_coord_xy[-1]))
            print('Coarse patchstar -> camera coordinate transform complete.')

            fine_pts2cam(self)
            print('Fine patchstar -> camera coordinate transform complete.')

            tip_z_found = pts2slsZ(self, show_result=False)
            print('Patchstar -> Slicescope Z coordinate transform complete.')
            input("Tip Found: {}?".format(tip_z_found))

            move_tip_2_high_mag_centre_on_low_mag(self, show_result=True)

            touch_water = low_mag_touch_water(self, show_result=True)

            #Put cross at proposed high magnification center x,y coordinate
            # Press ESC to close.
            #self.cam.video_cross(self.high_mag_centre_on_low_mag[0],self.high_mag_centre_on_low_mag[1])

            if touch_water:

                #Move condenser back to original position
                self.condenser.moveAbsolute(self.condenser.z_origin)

                #High Mag fine tip search Z and XY

                input('Switch to High Magnification????')

                move_high_mag_into_water(self)

                self.patchstar_list[idx].moveAbsolute(self.patchstar_list[idx].x, self.patchstar_list[idx].y,self.patchstar_list[idx].z + self.low_mag_2_high_mag_focal_plane)   #add the difference between low mag and high mag focal plane
                
                #Put cross at proposed high magnification center x,y coordinate
                # Press ESC to close.
                self.cam.video_cross(int(width/2),int(height/2))
                
                step = 150_00
                #self.patchstar_list[idx].moveAbsolute(self.patchstar_list[idx].x, self.patchstar_list[idx].y,self.patchstar_list[idx].z - int(step/2))
                #coord_img = pts_creepZ_image_coord(self, step, abs(int(step/10)))
                #self.img_proc.write_image_stack_to_file(coord_img['norm_img'], ['image_data', 'fine_tip_z_search'])

                high_mag_probe_search(self, step, abs(int(step/10)), show_result=False)

                start_live_update(self)
                input("End Live Update")
                end_live_update(self)

                #High Mag coarse calibration and first 4 points. Need to add more points in high_mag_fine_pts2cam and move slicescope relative down Z coord.

                high_mag_coarse_pts2cam(self, Point(self.patchstar_list[idx].tip_coord_xy[-1]))
                print('High Mag - Coarse patchstar -> camera coordinate transform complete.')

                high_mag_fine_pts2cam(self)
                print('High Mag - Fine patchstar -> camera coordinate transform complete.')



                print(f'Low  Mag pts2cam = \n {self.patchstar_list[idx].pts2cam["tran_xy"]}')
                print(f'High Mag pts2cam = \n {self.patchstar_list[idx].pts2cam_high["tran_xy"]}')

                print(f'Low  Mag cam2pts = \n {self.patchstar_list[idx].cam2pts["tran_xy"]}')
                print(f'High Mag cam2pts = \n {self.patchstar_list[idx].cam2pts_high["tran_xy"]}')

                start_live_update(self)
                input("End Live Update")
                end_live_update(self)

                self.condenser.moveAbsolute(self.condenser.z_max)

                #self.guess_screen_coord()


                break
            
            else:
                print(f'Patchstar{idx} tip did not touch water. Did not do High Magnification probe/tip search.')

                #Move probe away in the max x and z directions
                self.patchstar_list[idx].moveAbsolute(self.patchstar_list[idx].x_max, self.patchstar_list[idx].y_origin,self.patchstar_list[idx].z_max)

            #Update the coordinates for each patchstar with tip in focus and slicescope
            self.slicescope.coordinates()
            self.patchstar_list_slicescope_coordinates[idx] = [self.slicescope.x, self.slicescope.y, self.slicescope.z]

            self.patchstar_list[idx].coordinates()  #Update each patchstar x,y,z,a coordinates

            self.patchstar_list[idx].x_tip = self.patchstar_list[idx].x
            self.patchstar_list[idx].y_tip = self.patchstar_list[idx].y
            self.patchstar_list[idx].z_tip = self.patchstar_list[idx].z
            self.patchstar_list[idx].a_tip = self.patchstar_list[idx].a


            #Move probe away in the max x and z directions
            self.patchstar_list[idx].moveAbsolute(self.patchstar_list[idx].x_max, self.patchstar_list[idx].y_origin,self.patchstar_list[idx].z_max)

        cv.destroyAllWindows()

        #Move condenser back to original position
        self.condenser.moveAbsolute(self.condenser.z_origin)

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

    def guess_screen_coord(self):

        while True:

            screen_coord = list(map(int, input("Type in x y coordinates to put cross on the screen: ").strip().split()))[:2]

            frame = self.cam.get_frame()

            self.img_proc.cross(frame, screen_coord[0],screen_coord[1])

            if not screen_coord:
                break

            cv.imshow('Guess Screen Coordinates',frame)
            cv.waitKey(0)

    def patchstar_trajectory_angle(self):

        end_live_update(self)

        #Move each patchstar back into camera FOV and find the tip

        for idx in range(self.patchstar_num):
            self.patchstar_curr_idx = idx

            #Move to origin of patchstar (point 1)
            self.patchstar_list[idx].moveAbsolute(self.patchstar_list[idx].x_origin, self.patchstar_list[idx].y_origin,self.patchstar_list[idx].z_origin)

            #Move to x+delta, z+delta to get the angle of approach
            delta = 1000_00 
            self.patchstar_list[idx].moveRelative(0,0,0,delta,0,delta)

            self.patchstar_list[idx].coordinates()

            dz = self.patchstar_list[idx].z - self.patchstar_list[idx].z_origin
            dx = self.patchstar_list[idx].x - self.patchstar_list[idx].x_origin

            self.patchstar_list[idx].probe_angle = np.arctan2(dz,dx) * 180/np.pi

            print(self.patchstar_list[idx].probe_angle)


    def patch(self):
        pass        

    def set_top_speed(self, speed):

        self.live_flag = False

        speed = abs(speed)

        if speed > 4000:  #Keep max speed limit to 4000 to protect Patchstars. 
            speed = 4000

        #Set top speed for Slicescope and patchstars.
        command = f"TOP {speed}\r"
        
        self.slicescope.ser.write(command.encode('utf-8'))  #6000 is ok but better to use 4000. May be too fast for servos.
        byte_to_string = self.slicescope.ser.read_until(b'\r').decode('utf-8')
        print(f'slicescope top speed = {byte_to_string}')

        self.slicescope.ser.write(b'TOP\r')
        byte_to_string = self.slicescope.ser.read_until(b'\r').decode('utf-8')
        print(f'slicescope top speed = {byte_to_string}')

        for idx in range(self.patchstar_num):
            
            #Max speed limit is 4000 
            self.patchstar_list[idx].ser.write(command.encode('utf-8'))
            byte_to_string = self.patchstar_list[idx].ser.read_until(b'\r').decode('utf-8')
            print(f'patchstar {idx} top speed = {byte_to_string}')

    def check_top_speed(self):

        self.live_flag = False

        self.slicescope.ser.write(b'TOP\r')
        byte_to_string = self.slicescope.ser.read_until(b'\r').decode('utf-8')
        print(f'slicescope top speed = {byte_to_string}')

        for idx in range(self.patchstar_num):

            self.patchstar_list[idx].ser.write(b'TOP\r')
            byte_to_string = self.patchstar_list[idx].ser.read_until(b'\r').decode('utf-8')
            print(f'patchstar {idx} top speed = {byte_to_string}')

    def test_background_probe_histograms(self):

        end_live_update(self)

        #Move each patchstar back into camera FOV and find the tip

        for idx in range(self.patchstar_num):
            self.patchstar_curr_idx = idx

            while True:

                start_live_update(self)
                input('Clear camera FOV. Adjust lighting and iris.')
                end_live_update(self)

                background_img = self.cam.get_raw_frame(10)
                background_img_hist, background_idx_hist =  np.histogram(background_img.flatten(), bins=100)

                start_live_update(self)
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


def main():

    #Object called sp
    sp = SP()
    # SP caller decides which port to use as which instrument
    #for port, desc, hwid in sp.list_port_info:
    #    print(f"{port}: {desc} [{hwid}]")
    #Initialize all of the equipment
    sp.initialize()

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
    #start_live_update(sp)

    #end_live_update(sp)
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

    #Initial configuration
    #sp.configure()
    #sp.slicescope.moveAbsolute(sp.slicescope.x,sp.slicescope.y, int(sp.slicescope.z_max/3))

    #high_mag_probe_search(sp, 150_00, 15_00, show_result=True)

    sp.probe_adjustment()
    #sp.pseudo_probe_adjustment()

    sp.calibrate()
    #return


    #start_live_update(sp)
    #time.sleep(1)
    #end_live_update(sp)


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