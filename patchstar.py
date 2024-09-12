#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed August 9 12:21:00 2023

@author: Dr. Vincent Lee, Dr. WeiDa Zhang
Bioptix Ltd.

"""

# Use # for single line comment
# Use ''' ''' or """ """ as bookends for multiple line comments. Must be indented!

import serial 
import re
import numpy as np
import time

class Patchstar:

    # Class parameter

    # Instance method
    # Initialize instance with 'serial variable' in order to use serial.Serial functions
    def __init__(self,com_port):
        self.com_port = com_port
        self.ser = serial.Serial(port=self.com_port, baudrate=38400, bytesize=8, parity='N', stopbits=serial.STOPBITS_ONE, timeout=2)
        self.x = []
        self.y = []
        self.z = []
        self.a = []
        self.x_origin = 3_72 #8102
        self.y_origin = 17 #6591
        self.z_origin = 15_74 #-8
        self.a_origin = 9_34 #128756
        self.x_max = 10662_52 #1054764
        self.x_min = -10270_02 #-1038559
        self.y_max = 10664_55 #1054892
        self.y_min = -10378_27 #-1041709
        self.z_max = 10501_21 #1050184
        self.z_min = -10504_24 #-1050200
        self.a_max = 10480_81 #1485026
        self.a_min = -10454_68 #-1227512
        self.x_delta = 10467_76 #1046661
        self.y_delta = 10488_51 #1048300
        self.z_delta = 10502_84 #1050192
        self.a_max_delta = 10471_48 #950170
        self.a_min_delta = 10464_04 #1139899
        self.probe_x_max = 9712_70 #1053426
        self.probe_z_max = 3938_42 #1035098
        self.probe_x_min = -9698_36 #-1035980
        self.probe_z_min = -3904_15 #-1034474
        self.probe_angle = []
        self.x_scale = []
        self.y_scale = []
        self.z_scale = []
        self.a_scale = []
        self.x_tip = []
        self.y_tip = []
        self.z_tip = []
        self.a_tip = []
        self.a_angle = []
        self.tip_coord_quadrant = []
        self.tip_coord_angle = []
        self.tip_coord_xy = []
        self.tip_coord_z = []
        self.low_mag_water_z_coord = []

    def description(self):
        return "Micromanipulator"    

    def coordinates(self):

        #Get xyza coordinates
        self.ser.write(b'P ?\r')
        byte_to_string = self.ser.read_until(b'\r').decode('utf-8')
        coords = [ int(s) for s in re.findall( r'[-+]?\d+' , byte_to_string) ]

        self.x = coords[0]
        self.y = coords[1]
        self.z = coords[2]
        self.a = coords[3]

    def coordZ(self):

        #Get xyza coordinates
        self.ser.write(b'PZ\r')
        byte_to_string = self.ser.read_until(b'\r').decode('utf-8')
        coords = [ int(s) for s in re.findall( r'[-+]?\d+' , byte_to_string) ]

        self.z = coords[0]

        if self.z == self.z_max:
            print(f'Patchstar Z at MAX = {self.z_max}')
        if self.z == self.z_min:
            print(f'Patchstar Z at MIN = {self.z_min}')

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

        #check if max and min values are not empty lists
        if self.a_max:
            if self.a_min:
                #check if max and min values are not None
                if self.a_max is not None: 
                    if self.a_min is not None:
                        #check if max and min values are not NaN values
                        if not (np.isnan(self.a_max) or np.isnan(self.a_min)):

                            if not np.isnan(self.a):
                                if self.a > 0:
                                    self.a_delta = int(abs(self.a_max - self.a))
                                elif self.a < 0:
                                    self.a_delta = int(abs(self.a_min - self.a))
                                else:
                                    self.a_delta = int(abs((self.a_max - self.a_min)/2))
        else:
            self.a_max = None
            self.a_min = None
            self.a_delta = None

    def angle(self):

        self.ser.write(b'ANGLE\r')
        byte_to_string = self.ser.read_until(b'\r').decode('utf-8')
        probe_angle = int(re.search( r'[-+]?\d+' , byte_to_string).group())

        self.probe_angle = probe_angle

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

        #A axis scale
        self.ser.write(b'SCALE A\r')
        byte_to_string = self.ser.read_until(b'\r').decode('utf-8')
        a_scale = int(re.search( r'[-+]?\d+' , byte_to_string).group())

        self.a_scale = a_scale

        print(a_scale)

    def reset_coord_system(self):

        x_step = 100000_00
        y_step = 100000_00
        z_step = 100000_00

        tic = time.time()

        #Patchstar
        # X axis re-centering
        self.moveRelativeNoLimit(self.x,self.y,self.z,x_step,0,0)
        self.x_max = self.x
        self.moveRelativeNoLimit(self.x,self.y,self.z,-x_step,0,0)
        self.x_min = self.x

        self.x_delta = int(abs((self.x_max - self.x_min)/2))

        #Move patchstar back to center
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

        #Move patchstar back to center
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

        #Move patchstar back to center
        self.moveRelativeNoLimit(self.x,self.y,self.z,0,0,self.z_delta)
        self.z_origin = self.z

        #toc3 = time.time()
        #print(f'Z re-centering time = {toc3-toc2} seconds')

        # A axis re-centering
        self.coordinates()
        self.a_origin = self.a

        #Move X and Z together. Check common movement limits
        xz_upper_limit = min(self.x_max,self.z_max)

        self.approachRelativeNoLimit(self.a,xz_upper_limit)
        self.a_max = self.a
        self.probe_x_max = self.x
        self.probe_z_max = self.z
        self.approachRelativeNoLimit(self.a,-xz_upper_limit) #Move probe back to A origin

        xz_lower_limit = max(self.x_min,self.z_min)

        self.approachRelativeNoLimit(self.a,xz_lower_limit)
        self.a_min = self.a
        self.probe_x_min = self.x
        self.probe_z_min = self.z
        self.approachRelativeNoLimit(self.a,-xz_lower_limit) #Move probe back to A origin

        self.a_max_delta = int(abs(self.a_max - self.a_origin))
        self.a_min_delta = int(abs(self.a_origin - self.a_min))

        toc4 = time.time()
        #print(f'A re-centering time = {toc4-toc3} seconds')

        print(f'Total time reset coordinate system = {toc4-tic} seconds')

    def moveAbsolute(self,abs_x,abs_y,abs_z):  

    #Move to absolute position (x,y,z coordinates)
    #Set xyz coord ABSXYZ: ABS # # #
    #Set xyz coord ABSXY: ABS # #
    #Set a coord ABSZ x: ABSZ #

    #Once the center of the coordinate system is established, only use ABS to move to specified point.
    #Know the maximum limits of coordinate system and put safeguards on this limit.
    #X,Y,Z Limits +-10,000.00
    #Values are in hundredths of microns so 100.00 um is value = 10000

    #Limits of [X Y Z] = +-[10e5 10e5 10e5] 

        if abs_x > self.x_max:
            self.x = self.x_max
            print(f'Patchstar X at MAX = {self.x_max}')

        elif abs_x < self.x_min:
            self.x = self.x_min
            print(f'Patchstar X at MIN = {self.x_min}')

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
            print(f'Patchstar Y at MAX = {self.y_max}')

        elif abs_y < self.y_min:
            self.y = self.y_min
            print(f'Patchstar Y at MIN = {self.y_min}')

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
            print(f'Patchstar Z at MAX = {self.z_max}')

        elif abs_z < self.z_min:
            self.z = self.z_min
            print(f'Patchstar Z at MIN = {self.z_min}')

        else: 
            self.z = abs_z

        if self.z > 0:
            self.z_delta = int(abs(self.z_max - self.z))
        elif self.z < 0:
            self.z_delta = int(abs(self.z_min - self.z))
        else:
            self.z_delta = int(abs((self.z_max - self.z_min)/2))

        command = f"ABS {int(self.x)} {int(self.y)} {int(self.z)}\r"  #x,y,z coordinates 
        self.ser.write(command.encode('utf-8'))

        byte_to_string = self.ser.read_until(b'\r').decode('utf-8')
        #print(byte_to_string)

        print(f"Moved Micromanipulator to ABS coords = {int(self.x)} {int(self.y)} {int(self.z)}")   #Read the message/acknowledgment from the command

        self.wait()

        self.coordinates()

    def moveRelativeNoLimit(self,current_x,current_y,current_z,rel_x,rel_y,rel_z):
    #Move to NEW COORDINATES by new values relative to current coordinates

        command = f"REL {rel_x} {rel_y} {rel_z}\r"
        self.ser.write(command.encode('utf-8'))

        byte_to_string = self.ser.read_until(b'\r').decode('utf-8')
        #print(byte_to_string)

        #print(f"Moved Micromanipulator by REL coords = {rel_x} {rel_y} {rel_z}")
 
        self.wait()

        self.coordinates()

        #print(f"New Micromanipulator coordinates = {self.x} {self.y} {self.z} {self.a}")

    def moveRelative(self,current_x,current_y,current_z,rel_x,rel_y,rel_z):

    #Make sure to pass in global patchstar_x,y,z coordinate to this function and return new local_x,y,z value for assignment as patchstar_x,y,z

    #Move to relative position (x,y,z coordinates)
    #Set xyz coord RELXYZ: REL # # #
    #Set xyz coord RELXY: REL # #
    #Set a coord RELZ x: RELZ #

    #Limits of [X Y Z] = +-[10e5 10e5 10e5]  overall. Need to know current position to calculate margin
    #for relative coordinate

    #Set a coord RELX x: RELX #
    #Need to know current position to calculate margin for relative coordinate

        new_x = current_x + rel_x    

        if new_x > self.x_max:

            new_rel_x = int(self.x_max - current_x)
            self.x = self.x_max
            print(f'Patchstar X at MAX = {self.x_max}')

        elif new_x < self.x_min:

            new_rel_x = int(self.x_min - current_x)
            self.x = self.x_min
            print(f'Patchstar X at MIN = {self.x_min}')

        else:

            new_rel_x = rel_x
            self.x = new_x

        if self.x > 0:
            self.x_delta = int(abs(self.x_max - self.x))
        elif self.x < 0:
            self.x_delta = int(abs(self.x_min - self.x))
        else:
            self.x_delta = int(abs((self.x_max - self.x_min)/2))

        #print(f"Micromanipulator X coordinate is now = {self.x}")

    #Set a coord RELY x: RELY #

        new_y = current_y + rel_y   

        if new_y > self.y_max:
        
            new_rel_y = int(self.y_max - current_y)
            self.y = self.y_max
            print(f'Patchstar Y at MAX = {self.y_max}')

        elif new_y < self.y_min:
        
            new_rel_y = int(self.y_min - current_y)
            self.y = self.y_min
            print(f'Patchstar Y at MIN = {self.y_min}')

        else:
        
            new_rel_y = rel_y
            self.y = new_y

        if self.y > 0:
            self.y_delta = int(abs(self.y_max - self.y))
        elif self.y < 0:
            self.y_delta = int(abs(self.y_min - self.y))
        else:
            self.y_delta = int(abs((self.y_max - self.y_min)/2))

        #print(f"Micromanipulator Y coordinate is now = {self.y}")

    #Set a coord RELZ x: RELZ #

        new_z = current_z + rel_z        

        if new_z > self.z_max:
        
            new_rel_z = int(self.z_max - current_z)
            self.z = self.z_max
            print(f'Patchstar Z at MAX = {self.z_max}')

        elif new_z < self.z_min:
        
            new_rel_z = int(self.z_min - current_z)
            self.z = self.z_min
            print(f'Patchstar Z at MIN = {self.z_min}')

        else:
        
            new_rel_z = rel_z
            self.z = new_z

        if self.z > 0:
            self.z_delta = int(abs(self.z_max - self.z))
        elif self.z < 0:
            self.z_delta = int(abs(self.z_min - self.z))
        else:
            self.z_delta = int(abs((self.z_max - self.z_min)/2))

        #print(f"Micromanipulator Z coordinate is now = {self.z}")

    #Move to NEW COORDINATES by new values relative to current coordinates

        command = f"REL {int(new_rel_x)} {int(new_rel_y)} {int(new_rel_z)}\r"
        self.ser.write(command.encode('utf-8'))

        byte_to_string = self.ser.read_until(b'\r').decode('utf-8')
        #print(byte_to_string)

        print(f"Moved Micromanipulator by REL coords = {int(new_rel_x)} {int(new_rel_y)} {int(new_rel_z)}")
        print(f"New Micromanipulator coordinates = {int(self.x)} {int(self.y)} {int(self.z)}")

        self.wait()

        self.coordinates()

    def approachAbsolute(self,abs_a): 

    #Move to absolute coordinate A=XZ
    #Set a coord ABSA x: ABSA #

    #Upper limit is +14,500.00 or 14.5e5
    #Lower limit is -12,000.00 or -12e5

        if abs_a > self.a_max:
            self.a = self.a_max
            print(f'Patchstar A at MAX = {self.a_max}')

        elif abs_a < self.a_min:
            self.a = self.a_min
            print(f'Patchstar A at MIN = {self.a_min}')

        else: 
            self.a = abs_a

        if self.a > 0:
            self.a_delta = int(abs(self.a_max - self.a))
        elif self.a < 0:
            self.a_delta = int(abs(self.a_min - self.a))
        else:
            self.a_delta = int(abs((self.a_max - self.a_min)/2))

        command = f"ABSA {int(self.a)}\r"  #A=XZ coordinates 
        self.ser.write(command.encode('utf-8'))

        byte_to_string = self.ser.read_until(b'\r').decode('utf-8')
        #print(byte_to_string)

        print(f"Moved Probe to approach ABS = {int(self.a)}")   #Read the message/acknowledgment from the command

        self.wait()

        self.coordinates()

    def approachRelativeNoLimit(self,current_a,rel_a):  

    #Move to relative coordinate A=XZ
    #Set a coord RELA x: RELA #     

        command = f"RELA {int(rel_a)}\r"  #A=XZ coordinates 
        self.ser.write(command.encode('utf-8'))

        byte_to_string = self.ser.read_until(b'\r').decode('utf-8')
        #print(byte_to_string)

        #print(f"Moved Probe to approach REL = {rel_a}")   #Read the message/acknowledgment from the command

        self.wait()

        self.coordinates()

        #print(f"New Micromanipulator coordinates = {self.x} {self.y} {self.z} {self.a}")  #Read the message/acknowledgment from the command

    def approachRelative(self,current_a,rel_a):  

    #Move to relative coordinate A=XZ
    #Set a coord RELA x: RELA #

    #Upper limit is +14,500.00 or 14.5e5
    #Lower limit is -12,000.00 or -12e5

        new_a = current_a + rel_a        

        if new_a > self.a_max:
        
            new_rel_a = int(self.a_max - current_a)
            self.a = self.a_max
            print(f'Patchstar A at MAX = {self.a_max}')

        elif new_a < self.a_min:
        
            new_rel_a = int(self.a_min - current_a)
            self.a = self.a_min
            print(f'Patchstar A at MIN = {self.a_min}')

        else:
        
            new_rel_a = rel_a
            self.a = new_a

        if self.a > 0:
            self.a_delta = int(abs(self.a_max - self.a))
        elif self.a < 0:
            self.a_delta = int(abs(self.a_min - self.a))
        else:
            self.a_delta = int(abs((self.a_max - self.a_min)/2))

        command = f"RELA {int(new_rel_a)}\r"  #A=XZ coordinates 
        self.ser.write(command.encode('utf-8'))

        byte_to_string = self.ser.read_until(b'\r').decode('utf-8')
        #print(byte_to_string)

        print(f"Moved Probe to approach REL = {int(new_rel_a)}")   #Read the message/acknowledgment from the command
        print(f"Coordinate A = {int(self.a)}")   #Read the message/acknowledgment from the command

        self.wait()

        self.coordinates()

    def moveUp(self,current_z,rel_z):

        #Max is 12470.69 or 1.25e6
        #Min is -12470.00 or -1.25e6

        new_z = current_z + rel_z

        if rel_z > 0:

            if new_z > self.z_max:

                new_rel_z = int(self.z_max - current_z)
                self.z = self.z_max
                print(f'Patchstar Z at MAX = {self.z_max}')

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

            print(f"Micromanipulator Z coordinate is now = {int(self.z)}")

        self.wait()

        self.coordinates()

    def stepOut(self,patchstar_z,patchstar_a,step_out):
    #Objective step in or out. Step in closer to sample. Step out away from sample.

        #Absolute value of step value
        step_out = abs(step_out)

        self.ser.write(b'APPROACH\r')
        byte_to_string = self.ser.readline().decode('utf-8')
        status = int(re.search( r'[-+]?\d+' , byte_to_string).group()) #Search for numbers and convert MatchObject (whatever integer matches) as String.

        if (status == 1):
            #if APPROACH=1, then step can be used for A coordinate/direction
            
            new_a = patchstar_a + step_out

            if new_a > self.a_max:

                new_rel_a = int(self.a_max - patchstar_a)
                self.a = self.a_max
                print(f'Patchstar A at MAX = {self.a_max}')

            else:

                new_rel_a = step_out
                self.a = new_a

            self.a_delta = int(abs(self.a_max - self.a))

            command = f"SETSTEP {int(new_rel_a)}\r"  #Set step value.
            self.ser.write(command.encode('utf-8'))
            self.ser.write(b'STEP\r')    #Step out. Moves up in A=XZ direction by SETSTEP value

            byte_to_string = self.ser.read_until(b'\r').decode('utf-8')
            #print(byte_to_string)

            print(f"Moved Micromanipulator OUT by STEP = {int(new_rel_a)}")

            print(f"Micromanipulator A coordinate is now = {int(self.a)}")

        else:

            #if APPROACH=0, then step can be used only for Z coordinate/direction 

            new_z = patchstar_z + step_out

            if new_z > self.z_max:

                new_rel_z = int(self.z_max - patchstar_z)
                self.z = self.z_max
                print(f'Patchstar Z at MAX = {self.z_max}')

            else:

                new_rel_z = step_out
                self.z = new_z

            self.z_delta = int(abs(self.z_max - self.z))

            command = f"SETSTEP {int(new_rel_a)}\r"  #Set step value.
            self.ser.write(command.encode('utf-8'))
            self.ser.write(b'STEP\r')    #Step out. Moves up in Z direction by SETSTEP value

            byte_to_string = self.ser.read_until(b'\r').decode('utf-8')
            #print(byte_to_string)

            print(f"Moved Micromanipulator OUT by STEP = {int(new_rel_z)}")

            print(f"Micromanipulator Z coordinate is now = {int(self.z)}")

        self.wait()

        self.coordinates()

    def stepIn(self,patchstar_z,patchstar_a,step_in):
    #Objective step in or out. Step in closer to sample. Step out away from sample.

        #Absolute value of step value
        step_in = abs(step_in)
    
        self.ser.write(b'APPROACH\r')
        byte_to_string = self.ser.readline().decode('utf-8')
        status = int(re.search( r'[-+]?\d+' , byte_to_string).group()) #Search for numbers and convert MatchObject (whatever integer matches) as String.

        if (status == 1):
            #if APPROACH=1, then step can be used for A coordinate/direction
            
            new_a = patchstar_a - step_in

            if new_a < self.a_min:

                new_rel_a = int(patchstar_a - self.a_min)
                self.a = self.a_min
                print(f'Patchstar A at MIN = {self.a_min}')

            else:

                new_rel_a = step_in
                self.a = new_a

            self.a_delta = int(abs(self.a - self.a_min))

            command = f"SETSTEP {int(new_rel_a)}\r"  #Set step value.
            self.ser.write(command.encode('utf-8'))
            self.ser.write(b'STEP B\r')    #Step out. Moves up in A=XZ direction by SETSTEP value

            byte_to_string = self.ser.read_until(b'\r').decode('utf-8')
            #print(byte_to_string)

            print(f"Moved Micromanipulator IN by STEP = {int(new_rel_a)}")

            print(f"Micromanipulator A coordinate is now = {int(self.a)}")

        else:

            #if APPROACH=0, then step can be used only for Z coordinate/direction 
            
            new_z = patchstar_z - step_in

            if new_z < self.z_min:

                new_rel_z = int(patchstar_z - self.z_min)
                self.z = self.z_min
                print(f'Patchstar Z at MIN = {self.z_min}')

            else:

                new_rel_z = step_in
                self.z = new_z

            self.z_delta = int(abs(self.z - self.z_min))

            command = f"SETSTEP {int(new_rel_z)}\r"  #Set step value.
            self.ser.write(command.encode('utf-8'))
            self.ser.write(b'STEP B\r')    #Step out. Moves up in Z direction by SETSTEP value

            byte_to_string = self.ser.read_until(b'\r').decode('utf-8')
            #print(byte_to_string)

            print(f"Moved Micromanipulator OUT by STEP = {int(new_rel_z)}")

            print(f"Micromanipulator Z coordinate is now = {int(self.z)}")

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

    def creepA(self, distance, speed):

        #distance is in hundredths of microns
        #1_00 = 1.00 

        #speed is in micron/second

        if speed < 0:
            speed = 0

        command = f"CREEP A {int(distance)} {int(speed)}\r"
        self.ser.write(command.encode('utf-8'))

        byte_to_string = self.ser.read_until(b'\r').decode('utf-8')
        #print(byte_to_string)

        print(f"A creep distance = {int(distance)}")
        print(f"A creep speed = {int(speed)}")

    def set_top_speed(self, speed):

        #speed in microns/second. 

        speed = abs(speed)

        if speed > 4000:  #Keep max speed limit to 4000 to protect Patchstars. 
            speed = 4000

        #Set top speed for Slicescope and patchstars.
        command = f"TOP {speed}\r"

        #Max speed limit is 4000 
        self.ser.write(command.encode('utf-8'))
        byte_to_string = self.ser.read_until(b'\r').decode('utf-8')

    def check_top_speed(self):

        self.ser.write(b'TOP\r')
        byte_to_string = self.ser.read_until(b'\r').decode('utf-8')
        print(f'Check patchstar top speed = {byte_to_string}')