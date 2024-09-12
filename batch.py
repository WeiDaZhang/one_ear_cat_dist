# Dr. Vincent Lee
# batch.py
# Description:
# Class for Batch communications

import time
import win32com.client as win32
import win32gui

class Batch:

    #User opens Patchmaster Next and loads their configuration. Make sure the default setting is still b is the local shortcut in Patchmaster Next

    #####----------FUNCTIONS----------#####

    def windowEnumerationHandler(self, hwnd, top_windows):
        top_windows.append((hwnd, win32gui.GetWindowText(hwnd)))

    def batch_comm(self):

        #title = "PATCHMASTER NEXT"  # cycle all windows with this title
        title = "Configuration: 1_Defaults_Batch"  # cycle all windows with this title
        
        top_windows = []  # all open windows
        win32gui.EnumWindows(self.windowEnumerationHandler, top_windows)

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
