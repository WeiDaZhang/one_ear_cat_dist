#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thurs September 26 15:20:00 2024

@author: Dr. Vincent Lee
Bioptix Ltd.

"""

#Classes
from serial_port_tools import get_serial_ports

import serial
import re
import time

BAUD_RATE_LIST = [4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600]
WATCH_DOG_LIMIT = 3

def check_type(ser):
    port_name = ser.name
    baud_rate = ser.baudrate
    # Check the type of the device
    ser.write(b'TYPE\r')
    response = ser.read_until(b'\r').decode('utf-8')
    if response:
        if response == '3.11\r':
            print(f'{port_name} has baud {baud_rate} = Slicescope')
            return True, 'Slicescope'
        elif response == '3.12\r':
            print(f'{port_name} has baud {baud_rate} = Condenser')
            return True, 'Condenser'
        elif response == '3.05\r':
            print(f'{port_name} has baud {baud_rate} = Patchstar')
            return True, 'Patchstar'
        elif response[0] == 'E':
            print(f'{port_name} has baud {baud_rate}, Error Code in Response')
            return True, False
        else:
            print(f'{port_name} has baud {baud_rate} is unknown device')
            return True, 'Unknown Device'
    else:
        return False, 'No Response'

def check_instrument_type(port_info, instrument_list):
    port_name = port_info.name
    for baud_rate in BAUD_RATE_LIST:
        try:
            ser = serial.Serial(port = port_name, baudrate=baud_rate, timeout=1)
            watch_dog = 0
            while watch_dog < WATCH_DOG_LIMIT:
                watch_dog += 1
                response = check_type(ser)
                if not response[0]:
                    ser.close()
                    break
                elif not response[1]:
                    continue
                else:
                    instrument_list.append({'port_name': ser.name, 'baud_rate': ser.baudrate, 'type': response[1]})
                    ser.close()
                    return
            if ser.is_open:
                print(f'Port: {ser.name} keeps replying with Error Code')
                ser.close()
        except (serial.SerialTimeoutException, serial.serialutil.SerialException):
            print(f'{port_name} can\'t be opened.')
            break
    return 

def check_scientifica_instruments():
    instrument_list = []
    list_port_info = get_serial_ports()

    for idx in range(len(list_port_info)):
        check_instrument_type(list_port_info[idx], instrument_list)

    return instrument_list

def main():
    instrument_list = check_scientifica_instruments()
    print(instrument_list)    

if __name__ == '__main__':
    main()