# Dr. Vincent Lee
# heka.py
# Description:
# Class for HEKA communications

import time
import matplotlib.pyplot as plt
from threading import Thread

class Heka:

    def __init__(self):

        #PM Next file names
        #Mode Sender/Polling Sender: Sender.command
        #Mode Receiver/Polling Receiver: Receiv.answers

        self.filepath_s = r'C:\Users\Lab\Documents\HEKA\PM Next\Batch\Sender.command'
        self.filepath_r = r'C:\Users\Lab\Documents\HEKA\PM Next\Batch\Receiv.answers'

    def plot_graph(self, ax_handle, x, y):
        ax_handle.plot(x, y, 'k-')

    def initialize(self):

        with open(self.filepath_s,'wt') as fw:
            fw.write('0\n')

    def iv_monitor(self, batch_flag):

        x = []
        y = []
        z_mon = []

        if batch_flag == False:
            thread.join()

        #Setup figure properties for matplotlib
        fig = plt.figure()
        ax = fig.add_subplot(1, 1, 1)
        ax.set(title = 'Measured Impedance Z of Probe', xlabel = 'Index', ylabel = 'Impedance Z (Ohms)')
        plt.show(block=False)

        thread = Thread(target = self.plot_graph(ax, x, y))
        thread.setDaemon(True)
        thread.start()

        #Model cell: 10Mohm -> 7.5e5
        #Model cell: floating -> 5.5e7
        #Model cell: 500Mohm ->  2e7

        #Model cell values are 2 orders of magnitude different when not using a Test Pulse. 
        #This is enough for our measurements to differentiate the Seal vs Patch modes.

        cmd_idx = 1

        #File must be written to with only 1 command index and instruction every iteration. Each polling read will be with 1 set of instructions.
        #Must use Polling Receiver to ensure command index is accurate.

        while True:

            with open(self.filepath_s,'wt') as fw:
                line = [f'{cmd_idx}\n','Heartbeat\n','GetParameters Param-0, Param-1\n']
                fw.writelines(line)

            time.sleep(0.01)

            with open(self.filepath_r, 'rt') as fr:

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

            self.plot_graph(ax, x, y)
            plt.pause(0.01)

            cmd_idx = cmd_idx + 1



    def loop_monitor(self, batch_flag):

        x = []
        y = []
        z_mon = []

        #Setup figure properties for matplotlib
        fig = plt.figure()
        ax = fig.add_subplot(1, 1, 1)
        ax.set(title = 'Measured Impedance Z of Probe', xlabel = 'Index', ylabel = 'Impedance Z (Ohms)', ylim = (0,1e9))
        plt.show(block=False)

        thread = Thread(target = self.plot_graph(ax, x, y))
        thread.setDaemon(True)
        thread.start()

        with open(self.filepath_s,'wt') as fw:

            fw.write('1\n')
            file_path = 'OpenProtFile C:\\Users\\Lab\\Documents\\HEKA\\PM Next\\DefaultFiles\\Setup_Protocols\\SetupProt_Epc10.pro'
            fw.write(str(file_path) + '\n')
            fw.write('ExecuteProtocol Loop\n')

        time.sleep(0.1)

        with open(self.filepath_r, 'rt') as fr:
            fr.readlines()

        time.sleep(0.1)

        cmd_idx = 2

        #File must be written to with only 1 command index and instruction every iteration. Each polling read will be with 1 set of instructions.
        #Must use Polling Receiver to ensure command index is accurate.

        while batch_flag:

            with open(self.filepath_s,'wt') as fw:

                line = [f'{cmd_idx}\n','Heartbeat\n','GetParameters Param-10\n']
                fw.writelines(line)

            time.sleep(0.01)

            with open(self.filepath_r, 'rt') as fr:

                content_list = list(fr)

                if len(content_list) == 3:

                    line_segment = content_list[-1].strip().split(' ')
                    #z_mon = line_segment[-1]

                    z_mon.append(float(line_segment[-1]))

                    x.append(cmd_idx)
                    y.append(float(line_segment[-1]))

            self.plot_graph(ax, x, y)
            plt.pause(0.01)

            cmd_idx = cmd_idx + 1


    def Rpip_monitor(self, batch_flag):

        x = []
        y = []
        z_mon = []

        #Setup figure properties for matplotlib
        fig = plt.figure()
        ax = fig.add_subplot(1, 1, 1)
        ax.set(title = 'Measured Impedance Z of Probe', xlabel = 'Index', ylabel = 'Impedance Z (Ohms)', ylim = (0,1e9))
        plt.show(block=False)

        thread = Thread(target = self.plot_graph(ax, x, y))
        thread.setDaemon(True)
        thread.start()

        cmd_idx = 1

        #File must be written to with only 1 command index and instruction every iteration. Each polling read will be with 1 set of instructions.
        #Must use Polling Receiver to ensure command index is accurate.

        while batch_flag:

            with open(self.filepath_s,'wt') as fw:
                fw.write(f'{cmd_idx}\n')
                file_path = 'OpenProtFile C:\\Users\\Lab\\Documents\\HEKA\\PM Next\\DefaultFiles\\Setup_Protocols\\SetupProt_Epc10.pro'
                fw.write(str(file_path) + '\n')
                fw.write('ExecuteProtocol SaveRpip\n')

                line = ['Heartbeat\n','GetParameters Param-10\n']
                fw.writelines(line)

            time.sleep(0.01)

            with open(self.filepath_r,'rt') as fr:

                content_list = list(fr)

                if len(content_list) == 5:

                    line_segment = content_list[-1].strip().split(' ')
                    #z_mon = line_segment[-1]

                    z_mon.append(float(line_segment[-1]))

                    x.append(cmd_idx)
                    y.append(float(line_segment[-1]))

            self.plot_graph(ax, x, y)
            plt.pause(0.01)

            cmd_idx = cmd_idx + 1
