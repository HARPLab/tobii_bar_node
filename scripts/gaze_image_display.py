#!/usr/bin/env python

import cv2
import Tkinter as tk
import matplotlib.pyplot as plt
from collections import namedtuple


import rospy
from ibmmpy.msg import GazeData
import cv_bridge

GazePointInfo = namedtuple('GazePointInfo', ['x', 'y', 'time', 'idx'])


class GazeImageDraw:
    __LINGER_TIME__ = rospy.Duration(3.)
    __RADIUS__ = 10.
    __COLOR_MAP__ = plt.get_cmap('Reds')

    def __init__(self):
        self.window = tk.Tk()

        self.w, self.h = self.window.winfo_screenwidth(), self.window.winfo_screenheight()
        self.window.geometry("{}x{}".format(self.w, self.h))

        self.canvas = tk.Canvas(self.window)
        self.canvas.pack(expand=True, fill=tk.BOTH)

        self.gaze_sub = rospy.Subscriber('/gaze', GazeData, self.process_gaze)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.process_timer, oneshot=False)

        self._data = []        

    def process_gaze(self, gaze_msg):
        self._data.extend( ( GazePointInfo(msg.x, msg.y, msg.header.stamp, None) for msg in gaze_msg.world_data ) )
        self.redraw(gaze_msg.header.stamp)

    def process_timer(self, evt):
        self.redraw(evt.current_real)

    def redraw(tm):
        def process_points(pts):
            for pt in pts:
                delay = tm - pt.time

                if delay > GazeImageDraw.__LINGER_TIME__:
                    # remove old points
                    if pt.idx is not None:
                        self.canvas.delete(pt.idx)
                        pt.idx = None
                else:
                    # get new info
                    color = GazeImageDraw.__COLOR_MAP__( delay.to_sec() / GazeImageDraw.__LINGER_TIME__.to_sec() )
                    if pt.idx is None:
                        pt.idx = self.canvas.create_oval( pt.x - GazeImageDraw.__RADIUS__, pt.y - GazeImageDraw.__RADIUS__, 2*GazeImageDraw.__RADIUS__, 2*GazeImageDraw.__RADIUS__, fill=color )
                    else:
                        self.canvas.itemconfig(pt.idx, fill=color)
                    yield pt
        self._data = [ process_pts(self._data) ]
        self.window.update()

if __name__ == "__main__":
    rospy.init_node("gaze_image_display", anonymous=True)
    gaze_img_draw = GazeImageDraw()
    rospy.spin()



