#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed March 27 14:30:00 2024

@author: Dr. Vincent Lee
Bioptix Ltd.

"""
import time
import matplotlib.pyplot as plt
from threading import Thread

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
    ax.set(title = 'Measured Impedance Z of Probe', xlabel = 'Index', ylabel = 'Impedance Z (Ohms)')
    plt.show(block=False)

    thread = Thread(target = plot_graph(ax, x, y))
    thread.setDaemon(True)
    thread.start()

    #Model cell: 10Mohm -> 7.5e5
    #Model cell: floating -> 5.5e7
    #Model cell: 500Mohm ->  2e7

    #Model cell values are 2 orders of magnitude different when not using a Test Pulse. 
    #This is enough for our measurements to differentiate the Seal vs Patch modes.

    #EpcMaster file names
    #Mode Sender/Polling Sender: E9Batch.In
    #Mode Receiver/Polling Receiver: E9Batch.Out

    #Must use default folder. Can not specify Batch Comms folder in EpcMaster
    with open(r'C:\\Program Files (x86)\\HEKA\\EpcMaster\\E9Batch.In','wt') as fw:
        fw.write('0\n')

    time.sleep(0.1)

    print('Open EpcMaster. Under EPC10_USB, click Enable Batch Control.')
    print('Make sure to click on the EPC10_USB Amplifier window and keep this window ACTIVE to ensure the graph updates.')
    input('Press ENTER to continue.')

    cmd_idx = 1

    #File must be written to with only 1 command index and instruction every iteration. 
    #You must click on the EPC Amplifier window and keep the cursor on there for the parameters to update the graph. 
    #Otherwise the data points are constant from the last read.

    while batch_flag:

        with open(r'C:\\Program Files (x86)\\HEKA\\EpcMaster\\E9Batch.In','wt') as fw:
            line = [f'{cmd_idx}\n','Heartbeat\n','GetParameters Param-0, Param-1\n']
            fw.writelines(line)

        time.sleep(0.01)

        with open(r'C:\\Program Files (x86)\\HEKA\\EpcMaster\\E9Batch.Out', 'rt') as fr:

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
        plt.pause(0.1)

        cmd_idx = cmd_idx + 1

        if cmd_idx == 200:
            batch_flag = False

    print('Close Batch Control by clicking on Enable Batch Control. This closes and removes the E9Batch.In and E9Batch.Out files.')

if __name__ == '__main__':

    main()