#!/usr/bin/env python

import cv2
import Tkinter as tk
import matplotlib.pyplot as plt
import matplotlib.colors as colors
from collections import deque, namedtuple


import rospy
from ibmmpy.msg import GazeData
import cv_bridge

GazePointInfo = namedtuple('GazePointInfo', ['x', 'y', 'time', 'idx'])

class GazeImageDraw:
    __LINGER_TIME__ = rospy.Duration(1.)
    __RADIUS__ = 5
    __COLOR_MAP__ = plt.get_cmap('Reds')

    def __init__(self):
        self.window = tk.Tk()

        self.w, self.h = self.window.winfo_screenwidth(), self.window.winfo_screenheight()
        self.window.geometry("{}x{}".format(self.w, self.h))

        self.canvas = tk.Canvas(self.window)
        self.canvas.pack(expand=True, fill=tk.BOTH)

        self.gaze_sub = rospy.Subscriber('/gaze', GazeData, self.process_gaze)

        self._data = deque() 
        self._new_data = deque()    
        self.redraw()

    def process_gaze(self, gaze_msg):
        self._new_data.extend( ( GazePointInfo( int(msg.position.x * self.w), int(msg.position.y * self.h), msg.header.stamp, None) for msg in gaze_msg.world_data ) )

    def redraw(self):
        tm = rospy.get_rostime()

        # clear aged-out points
        while len(self._data) > 0 and self._data[0].time < tm - GazeImageDraw.__LINGER_TIME__:
            pt = self._data.popleft()
            if pt.idx is not None:
                self.canvas.delete(pt.idx)
        
        def get_opts(pt):
            delay = tm - pt.time
            fill = colors.rgb2hex(GazeImageDraw.__COLOR_MAP__( 1 - delay.to_sec() / GazeImageDraw.__LINGER_TIME__.to_sec() ))
            return {'fill': fill}

        # update the old points
        for pt in self._data:
            self.canvas.itemconfig(pt.idx, **get_opts(pt))

        # add any new points
        # use a new queue for thread-safety: we can't iterate _new_data in this thread
        # but pop() is thread-safe
        while len(self._new_data) > 0:
            pt = self._new_data.popleft()
            dims = pt.x - GazeImageDraw.__RADIUS__, pt.y - GazeImageDraw.__RADIUS__, pt.x + GazeImageDraw.__RADIUS__, pt.y + GazeImageDraw.__RADIUS__
            idx = self.canvas.create_oval(*dims, outline='', **get_opts(pt))
            self._data.append(GazePointInfo(pt.x, pt.y, pt.time, idx))
        
        self.window.after(100, self.redraw)


    def run(self):
        while not rospy.is_shutdown():
            self.window.update_idletasks()
            self.window.update()


if __name__ == "__main__":
    rospy.init_node("gaze_image_display", anonymous=True)
    gaze_img_draw = GazeImageDraw()
    gaze_img_draw.run()



