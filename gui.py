#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed June 19 18:00:00 2024

@author: Dr. Vincent Lee
Bioptix Ltd.

"""

#Classes
from slicescope import Slicescope
from condenser import Condenser
from patchstar import Patchstar
from pvcam import PVCAM
from tkgui import App

def main():

    print('-----MAIN-----')

    print('-----Slicescope-----')

    slicescope = Slicescope('COM4')

    slicescope.coordinates()
    print(f"Slicescope values X = {slicescope.x}, Y = {slicescope.y}, Z = {slicescope.z}")

    print('-----Condenser-----')

    condenser = Condenser('COM5')

    condenser.coordinates()
    print(f"Condenser value Z = {condenser.z}")

    print('-----Micromanipulator-----')

    patchstar1 = Patchstar('COM8')

    patchstar1.coordinates()
    print(f"Micromanipulator 1 values X = {patchstar1.x}, Y = {patchstar1.y}, Z = {patchstar1.z}, A = {patchstar1.a}")

    #patchstar2 = Patchstar('COM9')

    #patchstar2.coordinates()
    #print(f"Micromanipulator 2 values X = {patchstar2.x}, Y = {patchstar2.y}, Z = {patchstar2.z}, A = {patchstar2.a}")

    print('-----Camera-----')

    cam = PVCAM()

    #GUI
    root = App(slicescope, condenser, cam, patchstar1)#, patchstar2)
    root.mainloop()

    #Close and disconnect all equipment

    patchstar1.close()

    #patchstar2.close()

    cam.disconnect()

    condenser.close()

    slicescope.close()


if __name__ == '__main__':

    main()