#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed August 9 12:20:00 2023

@author: Dr. Vincent Lee, Dr. WeiDa Zhang
Bioptix Ltd.

"""

# Use # for single line comment
# Use ''' ''' or """ """ as bookends for multiple line comments. Must be indented!

import serial 
import re


class Condenser:

    # Class parameter
    
    # Instance method
    # Initialize instance with 'serial variable' in order to use serial.Serial functions
    def __init__(self,com_port):
        self.com_port = com_port
        self.ser = serial.Serial(port=com_port, baudrate=38400, bytesize=8, parity='N', stopbits=serial.STOPBITS_ONE, timeout=2)
        self.z = []
        self.z_origin = 1800_00
        self.z_max = 25000_00
        self.z_min = 0

    def description(self):
        return "Condenser"

    def coordinates(self):
        #Get Z coordinate
        self.ser.write(b'PZ\r')
        byte_to_string = self.ser.read_until(b'\r').decode('utf-8')
        #print(byte_to_string)

        self.z = int(byte_to_string)

    def moveAbsolute(self,abs_z):

    #Move to absolute position in z coordinate
    #Set a coord ABSZ x: ABSZ #
    #Limits of Z = 0-25000_00

        if abs_z > self.z_max:
            self.z = self.z_max

        elif abs_z < self.z_min:
            self.z = self.z_min

        else: 
            self.z = abs_z

        command = f"ABSZ {self.z}\r"  #z coordinates only. - is UP. + is DOWN.

        self.ser.write(command.encode('utf-8'))

        byte_to_string = self.ser.read_until(b'\r').decode('utf-8')
        #print(byte_to_string)

        print(f"Moved Condenser to ABS coord = {self.z}")

        self.wait()

        self.coordinates()

    def moveRelative(self,current_z,rel_z):
    #Move to relative position in z coordinate.

        new_z = current_z + rel_z

    #Set a coord RELZ x: RELZ #
    #Need to know current position to calculate margin for relative coordinate

        if new_z > self.z_max:
        
            new_rel_z = int(self.z_max - current_z)
            self.z = self.z_max

        elif new_z < self.z_min:
        
            new_rel_z = int(self.z_min - current_z)
            self.z = self.z_min

        else:
            new_rel_z = rel_z        
            self.z = new_z

    #Move to NEW COORDINATES by new values relative to current coordinates

        command = f"RELZ {new_rel_z}\r"

        self.ser.write(command.encode('utf-8'))

        byte_to_string = self.ser.read_until(b'\r').decode('utf-8')
        #print(byte_to_string)

        print(f"Moved Condenser by REL coord = {new_rel_z}")
        print(f"Condenser coordinate is now = {self.z}")

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
