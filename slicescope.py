#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed August 9 12:18:00 2023

@author: Dr. Vincent Lee, Dr. WeiDa Zhang
Bioptix Ltd.

"""

# Use # for single line comment
# Use ''' ''' or """ """ as bookends for multiple line comments. Must be indented!

import serial 
import re
import numpy as np
import time

class Slicescope:

    # Class parameter

    # Instance method
    # Initialize instance with 'serial variable' in order to use serial.Serial functions
    def __init__(self,com_port):
        self.com_port = com_port
        self.ser = serial.Serial(port=self.com_port, baudrate=38400, bytesize=8, parity='N', stopbits=serial.STOPBITS_ONE, timeout=2)
        self.x = []
        self.y = []
        self.z = []
        self.x_origin = 24_01 #14882
        self.y_origin = 1_20 #-1909
        self.z_origin = 500_00#5_35 #10800
        self.x_max = 25741_89 #2529934 
        self.x_min = -24558_69 #-2500170
        self.y_max = 22854_05 #2512914
        self.y_min = -27479_09 #-2516731
        self.z_max = 20443_41 #1256819
        self.z_min = -10000_00#-4569_14 #-1235219
        self.x_delta = 25152_24 #2515052
        self.y_delta = 25165_65 #2514822
        self.z_delta = 12507_42 #1246019
        self.x_scale = []
        self.y_scale = []
        self.z_scale = []
        self.x_tip = []
        self.y_tip = []
        self.z_tip = []
        self.high_mag_z_max = 2230_00
        self.high_mag_z_min = 0

    def description(self):
        return "Slicescope"

    def coordinates(self):

        #Get xyz coordinates
        self.ser.write(b'P\r')
        byte_to_string = self.ser.read_until(b'\r').decode('utf-8')
        coords = [ int(s) for s in re.findall( r'[-+]?\d+' , byte_to_string) ]

        self.x = coords[0]
        self.y = coords[1]
        self.z = coords[2]

    def deltas(self):

        #check if max and min values are not empty lists
        if self.x_max:
            if self.x_min:
                #check if max and min values are not None
                if self.x_max is not None: 
                    if self.x_min is not None:
                        #check if max and min values are not NaN values
                        if not (np.isnan(self.x_max) or np.isnan(self.x_min)):

                            if not np.isnan(self.x):
                                if self.x > 0:
                                    self.x_delta = int(abs(self.x_max - self.x))
                                elif self.x < 0:
                                    self.x_delta = int(abs(self.x_min - self.x))
                                else:
                                    self.x_delta = int(abs((self.x_max - self.x_min)/2))
        else:
            self.x_max = None
            self.x_min = None
            self.x_delta = None

        #check if max and min values are not empty lists
        if self.y_max:
            if self.y_min:
                #check if max and min values are not None
                if self.y_max is not None: 
                    if self.y_min is not None:

                        #check if max and min values are not NaN values
                        if not (np.isnan(self.y_max) or np.isnan(self.y_min)):

                            if not np.isnan(self.y):
                                if self.y > 0:
                                    self.y_delta = int(abs(self.y_max - self.y))
                                elif self.y < 0:
                                    self.y_delta = int(abs(self.y_min - self.y))
                                else:
                                    self.y_delta = int(abs((self.y_max - self.y_min)/2))
        else:
            self.y_max = None
            self.y_min = None
            self.y_delta = None

        #check if max and min values are not empty lists
        if self.z_max:
            if self.z_min:
                #check if max and min values are not None
                if self.z_max is not None: 
                    if self.z_min is not None:
                        #check if max and min values are not NaN values
                        if not (np.isnan(self.z_max) or np.isnan(self.z_min)):

                            if not np.isnan(self.z):
                                if self.z > 0:
                                    self.z_delta = int(abs(self.z_max - self.z))
                                elif self.z < 0:
                                    self.z_delta = int(abs(self.z_min - self.z))
                                else:
                                    self.z_delta = int(abs((self.z_max - self.z_min)/2))
        else:
            self.z_max = None
            self.z_min = None
            self.z_delta = None

    def scale(self):

        #X axis scale
        self.ser.write(b'SCALE X\r')
        byte_to_string = self.ser.read_until(b'\r').decode('utf-8')
        x_scale = int(re.search( r'[-+]?\d+' , byte_to_string).group())

        self.x_scale = x_scale

        print(x_scale)

        #Y axis scale
        self.ser.write(b'SCALE Y\r')
        byte_to_string = self.ser.read_until(b'\r').decode('utf-8')
        y_scale = int(re.search( r'[-+]?\d+' , byte_to_string).group())

        self.y_scale = y_scale

        print(y_scale)

        #Z axis scale
        self.ser.write(b'SCALE Z\r')
        byte_to_string = self.ser.read_until(b'\r').decode('utf-8')
        z_scale = int(re.search( r'[-+]?\d+' , byte_to_string).group())

        self.z_scale = z_scale

        print(z_scale)

    def reset_coord_system(self):

        x_step = 100000_00
        y_step = 100000_00
        z_step = 100000_00

        tic = time.time()

        #Slicescope
        # X axis re-centering
        self.moveRelativeNoLimit(self.x,self.y,self.z,x_step,0,0)
        self.x_max = self.x
        self.moveRelativeNoLimit(self.x,self.y,self.z,-x_step,0,0)
        self.x_min = self.x

        self.x_delta = int(abs((self.x_max - self.x_min)/2))
        
        #Move slicescope back to center
        self.moveRelativeNoLimit(self.x,self.y,self.z,self.x_delta,0,0)
        self.x_origin = self.x

        #toc1 = time.time()
        #print(f'X re-centering time = {toc1-tic} seconds')

        # Y axis re-centering
        self.moveRelativeNoLimit(self.x,self.y,self.z,0,y_step,0)
        self.y_max = self.y
        self.moveRelativeNoLimit(self.x,self.y,self.z,0,-y_step,0)
        self.y_min = self.y

        self.y_delta = int(abs((self.y_max - self.y_min)/2))

        #Move slicescope back to center
        self.moveRelativeNoLimit(self.x,self.y,self.z,0,self.y_delta,0)
        self.y_origin = self.y

        #toc2 = time.time()
        #print(f'Y re-centering time = {toc2-toc1} seconds')

        # Z axis re-centering
        self.moveRelativeNoLimit(self.x,self.y,self.z,0,0,z_step)
        self.z_max = self.z
        self.moveRelativeNoLimit(self.x,self.y,self.z,0,0,-z_step)
        self.z_min = self.z

        self.z_delta = int(abs((self.z_max - self.z_min)/2))

        #Move slicescope back to center
        self.moveRelativeNoLimit(self.x,self.y,self.z,0,0,self.z_delta)
        self.z_origin = self.z

        toc3 = time.time()
        #print(f'Z re-centering time = {toc3-toc2} seconds')

        #Set High Magnification lower limit (add 4mm of clearance to enable probe to fit. Distance is a little more than the thickness of the bottom of the chamber)
        self.high_mag_z_min = int(self.z_min + 4000_00)

        print(f'Total time for reset coordinate system = {toc3-tic} seconds')

    def moveAbsolute(self,abs_x,abs_y,abs_z):   

    ###### Note: Be sure to check limit when using 40x objective. This must be -4000.00 minimum.

    #Move to absolute position (x,y,z coordinates)
    #Set xyz coord ABSXYZ: ABS # # #
    #Set xyz coord ABSXY: ABS # #
    #Set a coord ABSZ x: ABSZ #

    #Once the center of the coordinate system is established, only use ABS to move to specified point.
    #Know the maximum limits of coordinate system and put safeguards on this limit.
    #X,Y Limits +-25,000.00
    #Z Limits +- 12,500.00
    #Values are in hundredths of microns so 100.00 um is value = 10000

    #Limits of [X Y Z] = +-[25e5 25e5 12.5e5] 

        if abs_x > self.x_max:
            self.x = self.x_max
            print(f'Slicescope X at MAX = {self.x_max}')

        elif abs_x < self.x_min:
            self.x = self.x_min
            print(f'Slicescope X at MIN = {self.x_min}')

        else: 
            self.x = abs_x

        if self.x > 0:
            self.x_delta = int(abs(self.x_max - self.x))
        elif self.x < 0:
            self.x_delta = int(abs(self.x_min - self.x))
        else:
            self.x_delta = int(abs((self.x_max - self.x_min)/2))

        if abs_y > self.y_max:
            self.y = self.y_max
            print(f'Slicescope Y at MAX = {self.y_max}')

        elif abs_y < self.y_min:
            self.y = self.y_min
            print(f'Slicescope Y at MIN = {self.y_min}')

        else: 
            self.y = abs_y

        if self.y > 0:
            self.y_delta = int(abs(self.y_max - self.y))
        elif self.y < 0:
            self.y_delta = int(abs(self.y_min - self.y))
        else:
            self.y_delta = int(abs((self.y_max - self.y_min)/2))

        if abs_z > self.z_max:
            self.z = self.z_max
            print(f'Slicescope Z at MAX = {self.z_max}')

        elif abs_z < self.z_min:
            self.z = self.z_min
            print(f'Slicescope Z at MIN = {self.z_min}')

        else: 
            self.z = abs_z

        if self.z > 0:
            self.z_delta = int(abs(self.z_max - self.z))
        elif self.z < 0:
            self.z_delta = int(abs(self.z_min - self.z))
        else:
            self.z_delta = int(abs((self.z_max - self.z_min)/2))

        command = f"ABS {int(self.x)} {int(self.y)} {int(self.z)}\r" #x,y,z coordinates 
        self.ser.write(command.encode('utf-8'))

        byte_to_string = self.ser.read_until(b'\r').decode('utf-8')
        #print(byte_to_string)

        print(f"Moved Slicescope to ABS coords (X Y Z) = {int(self.x)} {int(self.y)} {int(self.z)}")

        self.wait()

        self.coordinates()

    def moveRelativeNoLimit(self,current_x,current_y,current_z,rel_x,rel_y,rel_z):
    #Move to NEW COORDINATES by new values relative to current coordinates

        command = f"REL {int(rel_x)} {int(rel_y)} {int(rel_z)}\r"
        self.ser.write(command.encode('utf-8'))

        byte_to_string = self.ser.read_until(b'\r').decode('utf-8')
        #print(byte_to_string)

        #print(f"Moved Slicescope by REL coords = {rel_x} {rel_y} {rel_z}")
 
        self.wait()

        self.coordinates()

        #print(f"New Slicescope coordinates = {self.x} {self.y} {self.z}")

    def moveRelative(self,current_x,current_y,current_z,rel_x,rel_y,rel_z):

    ###### Note: Be sure to check limit when using 40x objective. This must be -4000.00 minimum.
    #Make sure to pass in global slicescope_x,y,z coordinates to this function and return new local_x,y,z values for assignment as slicescope_x,y,z

    #Move to relative position (x,y,z coordinates)
    #Set xyz coord RELXYZ: REL # # #
    #Set xyz coord RELXY: REL # #
    #Set a coord RELZ x: RELZ #

    #Limits of [X Y Z] = +-[25e5 25e5 12.5e5]  overall. Need to know current position to calculate margin
    #for relative coordinate

    #Set a coord RELX x: RELX #
    #Need to know current position to calculate margin for relative coordinate

        new_x = current_x + rel_x    
    
        if new_x > self.x_max:
        
            new_rel_x = int(self.x_max - current_x)
            self.x = self.x_max
            print(f'Slicescope X at MAX = {self.x_max}')

        elif new_x < self.x_min:
        
            new_rel_x = int(self.x_min - current_x)
            self.x = self.x_min
            print(f'Slicescope X at MIN = {self.x_min}')

        else:
        
            new_rel_x = rel_x
            self.x = new_x

        #print(f"Slicescope X coordinate is now = {self.x}")

        if self.x > 0:
            self.x_delta = int(abs(self.x_max - self.x))
        elif self.x < 0:
            self.x_delta = int(abs(self.x_min - self.x))
        else:
            self.x_delta = int(abs((self.x_max - self.x_min)/2))

    #Set a coord RELY x: RELY #

        new_y = current_y + rel_y   

        if new_y > self.y_max:
        
            new_rel_y = int(self.y_max - current_y)
            self.y = self.y_max
            print(f'Slicescope Y at MAX = {self.y_max}')

        elif new_y < self.y_min:
        
            new_rel_y = int(self.y_min - current_y)
            self.y = self.y_min
            print(f'Slicescope Y at MIN = {self.y_min}')

        else:
        
            new_rel_y = rel_y
            self.y = new_y

        #print(f"Slicescope Y coordinate is now = {self.y}")

        if self.y > 0:
            self.y_delta = int(abs(self.y_max - self.y))
        elif self.y < 0:
            self.y_delta = int(abs(self.y_min - self.y))
        else:
            self.y_delta = int(abs((self.y_max - self.y_min)/2))

    #Set a coord RELZ x: RELZ #

        new_z = current_z + rel_z        

        if new_z > self.z_max:
        
            new_rel_z = int(self.z_max - current_z)
            self.z = self.z_max
            print(f'Slicescope Z at MAX = {self.z_max}')

        elif new_z < self.z_min:
        
            new_rel_z = int(self.z_min - current_z)
            self.z = self.z_min
            print(f'Slicescope Z at MIN = {self.z_min}')

        else:
        
            new_rel_z = rel_z
            self.z = new_z

        #print(f"Slicescope Z coordinate is now = {self.z}")

        if self.z > 0:
            self.z_delta = int(abs(self.z_max - self.z))
        elif self.z < 0:
            self.z_delta = int(abs(self.z_min - self.z))
        else:
            self.z_delta = int(abs((self.z_max - self.z_min)/2))

    #Move to NEW COORDINATES by new values relative to current coordinates

        command = f"REL {int(new_rel_x)} {int(new_rel_y)} {int(new_rel_z)}\r"
        self.ser.write(command.encode('utf-8'))

        byte_to_string = self.ser.read_until(b'\r').decode('utf-8')
        #print(byte_to_string)

        #print(f"Moved Slicescope by REL coords = {int(new_rel_x)} {int(new_rel_y)} {int(new_rel_z)}")
        #print(f"New Slicescope coordinates = {int(self.x)} {int(self.y)} {int(self.z)}")

        self.wait()

        self.coordinates()

    def moveUp(self,current_z,rel_z):

        ###### Note: Be sure to check limit when using 40x objective. This must be -4000.00 minimum.

        #Max is 12470.69 or 1.25e6
        #Min is -12470.00 or -1.25e6

        new_z = current_z + rel_z

        if rel_z > 0:
        
            if new_z > self.z_max:

                new_rel_z = int(self.z_max - current_z)
                self.z = self.z_max
                print(f'Slicescope Z at MAX = {self.z_max}')

            else:
            
                new_rel_z = rel_z
                self.z = new_z

            self.z_delta = int(abs(self.z_max - self.z))

            #Create string with command, then convert to byte to write to serial.

            command = f"OBJLIFT {int(new_rel_z)}\r"  #Values in OBJLIFT can be negative.
            self.ser.write(command.encode('utf-8'))
            self.ser.write(b'OBJU\r')  #Moves like RELZ. 

            byte_to_string = self.ser.read_until(b'\r').decode('utf-8')
            #print(byte_to_string)

            #print(f"Slicescope Z coordinate is now = {int(self.z)}")

            self.wait()

            self.coordinates()

    def stepOut(self,slicescope_z,step_out):

    ###### Note: Be sure to check limit when using 40x objective. This must be -4000.00 minimum.

    #Objective step in or out. Step in closer to sample. Step out away from sample.
    #Pass in global slicescope_z coordinate to this function and return new local_z value for assignment

        #Absolute value of step value
        step_out = abs(step_out)

        new_z = slicescope_z + step_out

        if new_z > self.z_max:

            new_rel_z = int(self.z_max - slicescope_z)
            self.z = self.z_max
            print(f'Slicescope Z at MAX = {self.z_max}')

        else:

            new_rel_z = step_out
            self.z = new_z

        self.z_delta = int(abs(self.z_max - self.z))

        command = f"SETSTEP {int(new_rel_z)}\r"  #Set step value.
        self.ser.write(command.encode('utf-8'))
        self.ser.write(b'STEP\r')    #Step out. Moves up in Z direction by SETSTEP value

        byte_to_string = self.ser.read_until(b'\r').decode('utf-8')
        #print(byte_to_string)

        #print(f"Moved Slicescope OUT by STEP = {int(new_rel_z)}")

        #print(f"Slicescope Z coordinate is now = {int(self.z)}")

        self.wait()

        self.coordinates()

    def stepIn(self,slicescope_z,step_in):

    ###### Note: Be sure to check limit when using 40x objective. This must be -4000.00 minimum.

    #Objective step in or out. Step in closer to sample. Step out away from sample.
    #Pass in global slicescope_z coordinate to this function and return new local_z value for assignment

        #Absolute value of step value
        step_in = abs(step_in)

        new_z = slicescope_z - step_in

        if new_z < self.z_min:

            new_rel_z = int(slicescope_z - self.z_min)
            self.z = self.z_min
            print(f'Slicescope Z at MIN = {self.z_min}')

        else:

            new_rel_z = step_in
            self.z = new_z

        self.z_delta = int(abs(self.z - self.z_min))

        command = f"SETSTEP {int(new_rel_z)}\r"  #Set step value.
        self.ser.write(command.encode('utf-8'))
        self.ser.write(b'STEP B\r')   #Step in. Moves down in Z direction by SETSTEP value

        byte_to_string = self.ser.read_until(b'\r').decode('utf-8')
        #print(byte_to_string)

        #print(f"Moved Slicescope IN by STEP = {int(new_rel_z)}")

        #print(f"Slicescope Z coordinate is now = {int(self.z)}")

        self.wait()

        self.coordinates()

    def stop(self):
        #Stop object
        self.ser.write(b'STOP S\r')

        byte_to_string = self.ser.read_until(b'\r').decode('utf-8')
        #print(byte_to_string)

    def status(self):

        #Status (objective is stationary=0 or moving=1,2)
        self.ser.write(b'S\r')
        byte_to_string = self.ser.read_until(b'\r').decode('utf-8')
        #print(byte_to_string)
        status = int(re.search( r'[-+]?\d+' , byte_to_string).group()) #Byte converted to String converted to Integer.
                                                                       #Search for numbers and convert MatchObject (whatever integer matches) as String.

        return status

    def wait(self):
        #Wait for command to finish executing and instance stopped moving
        #Status (objective is stationary=0 or moving=1 or 2)

        status = self.status()

        while (status != 0):

            status = self.status()
            #print(status)

            if (status == 0):
                break

    def close(self):
        #Close connection
        self.ser.close()
        print(f"Connected to {self.ser} = {self.ser.is_open}")

    def creepX(self, distance, speed):

        #distance is in hundredths of microns
        #1_00 = 1.00 

        #speed is in micron/second

        if speed < 0:
            speed = 0

        command = f"CREEP X {int(distance)} {int(speed)}\r"
        self.ser.write(command.encode('utf-8'))

        byte_to_string = self.ser.read_until(b'\r').decode('utf-8')
        #print(byte_to_string)

        print(f"X creep distance = {int(distance)}")
        print(f"X creep speed = {int(speed)}")

    def creepY(self, distance, speed):

        #distance is in hundredths of microns
        #1_00 = 1.00 

        #speed is in micron/second

        if speed < 0:
            speed = 0

        command = f"CREEP Y {int(distance)} {int(speed)}\r"
        self.ser.write(command.encode('utf-8'))

        byte_to_string = self.ser.read_until(b'\r').decode('utf-8')
        #print(byte_to_string)

        print(f"Y creep distance = {int(distance)}")
        print(f"Y creep speed = {int(speed)}")

    def creepZ(self, distance, speed):

        #distance is in hundredths of microns
        #1_00 = 1.00 

        #speed is in micron/second

        if speed < 0:
            speed = 0

        command = f"CREEP Z {int(distance)} {int(speed)}\r"
        self.ser.write(command.encode('utf-8'))

        byte_to_string = self.ser.read_until(b'\r').decode('utf-8')
        #print(byte_to_string)

        print(f"Z creep distance = {int(distance)}")
        print(f"Z creep speed = {int(speed)}")
