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

def batch_comm():

    #from AppActivate.py
    title = "PATCHMASTER NEXT"  # cycle all windows with this title

    top_windows = []  # all open windows
    win32gui.EnumWindows(windowEnumerationHandler, top_windows)

    winlst = []  # windows to cycle through
    for i in top_windows:  # all open windows
        if i[1] == title:
            winlst.append(i)

    for w in winlst:  # each window with selected title
        shell = win32.Dispatch("WScript.Shell")  # set focus on desktop
        shell.SendKeys('%')  # Alt key
        win32gui.SetForegroundWindow(w[0]) # bring to front, activate
        time.sleep(0.1)

        # WinShell presses 'b' for batch communications shortcut in Patchmaster Next. 
        # Make sure batch communication b shortcut is set to GLOBAL.
        # b can be pressed again to close batch communication.
        shell.SendKeys('b')  # b key
        time.sleep(0.1)

def plot_graph(ax_handle, x, y):
    ax_handle.plot(x, y, 'k-')

def main():

    batch_flag = True

    #User opens Patchmaster Next and loads their configuration. Make sure b is the GLOBAL shortcut in Patchmaster Next

    x = []
    y = []

    #Setup figure properties for matplotlib
    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1)
    ax.set(title = 'Measured Impedance Z of Probe', xlabel = 'Index', ylabel = 'Impedance Z (Ohms)')
    plt.show(block=False)

    thread = Thread(target = plot_graph(ax, x, y))
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
    batch_comm()

    time.sleep(0.1)

    cmd_idx = 1

    #File must be written to with only 1 command index and instruction every iteration. Each polling read will be with 1 set of instructions.
    #Must use Polling Receiver to ensure command index is accurate.

    while batch_flag:

        with open(r'C:\\Users\\Lab\\Documents\\HEKA\\PM Next\\Batch\\Sender.command','wt') as fw:
            line = [f'{cmd_idx}\n','Heartbeat\n','GetParameters Param-0, Param-1\n']
            fw.writelines(line)

        time.sleep(0.01)

        with open(r'C:\\Users\\Lab\\Documents\\HEKA\\PM Next\\Batch\\Receiv.answers', 'rt') as fr:

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

        plot_graph(ax, x, y)
        plt.pause(0.01)

        cmd_idx = cmd_idx + 1

        if cmd_idx == 200:
            batch_flag = False

    batch_comm()


if __name__ == '__main__':

    main()