import os
import string
import numpy as np
import cv2 as cv
import threading
import time



class Image_window:
    def __init__(self, title = "Image", size = (1024, 1024), target_frame_rate = 10):
        self.title = title
        self.size = size
        self.frame = np.zeros(size)
        self.overlay = np.ones(size)
        self.click_coord = [0, 0]
        self.previous_click_coord = [0, 0]
        self.target_frame_rate = target_frame_rate
        self.thread = []

    def set_live_thread(self):
        self.thread = threading.Thread(target = self.show_live)

    def wait_window_ready(self):
        while not cv.getWindowProperty(self.title, cv.WND_PROP_VISIBLE):
            continue

    def set_mouse_response(self):
        cv.setMouseCallback(self.title, self.mouse_response)

    def click_coord_update(self):
        self.previous_click_coord = self.click_coord.copy()

    def mouse_response(self, event, x, y, flags, param):
        # Mouse Left Button Press and Drag calls get coordinate
        if event == cv.EVENT_LBUTTONDOWN:
            self.get_click_coord(x, y)
        elif event == cv.EVENT_LBUTTONUP:
            self.get_click_coord(x, y)
        elif event == cv.EVENT_MOUSEMOVE:
            if flags:
                if flags == cv.EVENT_FLAG_LBUTTON:
                    self.get_click_coord(x, y)

    def get_click_coord(self, x, y):
        self.click_coord = [x, y]

    def show_live(self):
        while True:
            frame_norm = self.frame * self.overlay
            if not np.max(frame_norm) == 0:
                frame_norm = frame_norm / np.max(frame_norm)
            frame_norm = np.round(frame_norm * 255).astype(np.uint8)
            cv.imshow(self.title, frame_norm)

            # Update at defined frame rate and Wait for ESC to exit
            if cv.waitKey(int(1000/self.target_frame_rate)) == 27:
                break

    def gen_rand_bw_frame(self):
        self.frame = np.floor(np.random.rand(self.size[0], self.size[1]) * 256).astype(np.uint8)

    def clear_overlay(self):
        self.overlay = np.ones(self.size)

    def add_overlay(self, input_overlay, inverse = True):
        input_overlay = input_overlay - np.min(input_overlay)
        input_overlay = input_overlay / np.max(input_overlay)
        if inverse:
            input_overlay = np.max(input_overlay) - input_overlay
        self.overlay += input_overlay
        self.overlay = self.overlay / np.max(self.overlay)
