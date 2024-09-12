#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed March 27 14:30:00 2024

@author: Dr. Vincent Lee
Bioptix Ltd.

"""
import time
import matplotlib.pyplot as plt

import win32com.client as win32
import win32gui
from threading import Thread

#from AppActivate.py
def windowEnumerationHandler(hwnd, top_windows):
    top_windows.append((hwnd, win32gui.GetWindowText(hwnd)))

def childWindowEnumerationHandler(hwnd, child_windows):
    child_windows.append((hwnd, win32gui.GetWindowText(hwnd)))

def batch_comm():
    #from AppActivate.py
    title = "PatchMaster"  # cycle all windows with this title <<<< Case Sensitive!!!

    top_windows = []  # all open windows
    win32gui.EnumWindows(windowEnumerationHandler, top_windows)

    #print(top_windows)

    child_windows = []
    #i[0] = hwnd in top_windows list, i[1] = Window Text in top_windows list
    for i in top_windows:
        if i[1] == title:
            win32gui.EnumChildWindows(i[0],childWindowEnumerationHandler,child_windows)

    #print(child_windows)

    winlst = []  # windows to cycle through
    for j in child_windows:  # all child windows
        if "Amplifier" in j[1]:
            winlst.append(j)

    #print(winlst)

    for w in winlst:  # each window with selected title
        shell = win32.Dispatch("WScript.Shell")  # set focus on desktop
        shell.SendKeys('%')  # Alt key
        win32gui.SetForegroundWindow(w[0]) # bring to front, activate
        time.sleep(0.1)
        #This brings the Amplifier window to the foreground and keep it active. 

        #Do not need to use SPACE bar to activate Amplifier window. Above code is more stable. Use window handle with Amplifier in title.
        # WinShell presses ' ' {SPACE} for batch communications shortcut in PatchMaster. 
        # The SPACE key will bring the Amplifier window to the foreground and keep it active. 
        # SPACE key will toggle between Configuration window and Amplifier window.
        #shell.SendKeys(' ')  # SPACE key
        #shell.SendKeys(' ')  # SPACE key
        #time.sleep(0.1)

    return winlst


def plot_graph(ax_handle, x, y):
    ax_handle.plot(x, y, 'k-')

def main():

    batch_flag = True

    x = []
    y = []
    z_mon = []

    #Setup figure properties for matplotlib
    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1)
    ax.set(title = 'Measured Impedance Z of Probe', xlabel = 'Index', ylabel = 'Impedance Z (Ohms)', ylim = (0,1e9))
    plt.show(block=False)

    thread = Thread(target = plot_graph(ax, x, y))
    thread.setDaemon(True)
    thread.start()

    #Model cell: 10Mohm -> 6.5e5
    #Model cell: floating -> 4.5e7
    #Model cell: 500Mohm ->  1.8e7

    #Model cell values are 2 orders of magnitude different when not using a Test Pulse. 
    #This is enough for our measurements to differentiate the Seal vs Patch modes.

    #Patchmaster file names
    #Mode Sender/Polling Sender: E9Batch.In
    #Mode Receiver/Polling Receiver: E9Batch.Out

    with open(r'C:\\Program Files (x86)\\HEKA\\PatchMaster\\Batch\\E9Batch.In','wt') as fw:
        fw.write('0\n')

    time.sleep(0.1)

    print('Open Patchmaster. Under Windows, open Configuration.')
    print('Go to Configuration/Files. Setup the filepath for Batch Path.')
    print('Go to Configuration/General. In Batch Communication, Enable Polling and Receiver.')
    print('The Configuration window can remain open during this time and the graph will update.')
    input('Press ENTER to continue.')

    cmd_idx = 1

    #File must be written to with only 1 command index and instruction every iteration. Each polling read will be with 1 set of instructions.
    #Must use Polling Receiver to ensure command index is accurate.

    child_lst = batch_comm()  #child_lst should only have Amplifier window in this list. 

    while batch_flag:

        #First bring Amplifier window to foreground and keep active to read the Impedance.
        for c in child_lst:  # each window with selected title
            shell = win32.Dispatch("WScript.Shell")  # set focus on desktop
            shell.SendKeys('%')  # Alt key
            win32gui.SetForegroundWindow(c[0]) # bring to front, activate
            time.sleep(0.1)

        with open(r'C:\\Program Files (x86)\\HEKA\\PatchMaster\\Batch\\E9Batch.In','wt') as fw:
            line = [f'{cmd_idx}\n','Heartbeat\n','ExecuteProtocol SaveRpip\n','GetParameters Param-10\n']
            fw.writelines(line)

        time.sleep(0.01)

        with open(r'C:\\Program Files (x86)\\HEKA\\PatchMaster\\Batch\\E9Batch.Out','rt') as fr:

            content_list = list(fr)

            if len(content_list) == 4:

                line_segment = content_list[-1].strip().split(' ')
                #z_mon = line_segment[-1]

                z_mon.append(float(line_segment[-1]))

                x.append(cmd_idx)
                y.append(float(line_segment[-1]))

        plot_graph(ax, x, y)
        plt.pause(0.1)

        cmd_idx = cmd_idx + 1

        if cmd_idx == 200:
            batch_flag = False

    print('In the Configuration window, click Batch Communication - Disable.')
    print('This stops the Batch processing and closes and removes the E9Batch.In and E9Batch.Out files.')

if __name__ == '__main__':

    main()