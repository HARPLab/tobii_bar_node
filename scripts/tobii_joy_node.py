#!/bin/env python3

import cv2
import numpy as np
import rospy

import ibmmpy.msg
import sensor_msgs.msg

class ControlButton:
    def __init__(self, x1, y1, x2, y2, ax_idx, scale):
        self._pt1 = (x1, y1)
        self._pt2 = (x2, y2)
        self._ax_idx = ax_idx
        self._scale = scale

    def draw(self, frame, is_active=False):
        cv2.rectangle(frame, self._pt1, self._pt2, (255, 100, 0), -1 if is_active else 10)
    
    def check(self, pt):
        return (self._pt1[0] >= pt[0] and
                self._pt1[1] >= pt[1] and
                self._pt2[0] <= pt[0] and
                self._pt2[1] <= pt[1])
    
    def update(self, msg):
        msg.axis[self._ax_idx] = self._scale

WIDTH, HEIGHT = (1280, 720)

CONTROL_BUTTONS = [
    ControlButton(int(WIDTH*.1), int(HEIGHT*.4), int(WIDTH*.45), int(HEIGHT*.6), 0, -1), # left
    ControlButton(int(WIDTH*.55), int(HEIGHT*.3), int(WIDTH*.9), int(HEIGHT*.7), 0, 1), # right
    ControlButton(int(WIDTH*.3), int(HEIGHT*.1), int(WIDTH*.7), int(HEIGHT*.5), 1, 1), # up
    ControlButton(int(WIDTH*.3), int(HEIGHT*.55), int(WIDTH*.7), int(HEIGHT*.9), 1, -1), # down
]


class GazeToJoy:
    def __init__(self):
        self._pub = rospy.Publisher("/joy", sensor_msgs.msg.Joy, queue_size=1)
        self._sub = rospy.Subscriber("/gaze", ibmmpy.msg.GazeData, self._gaze_cb)
        self._win_name = "Controller"

        cv2.namedWindow(self._win_name, cv2.WINDOW_GUI_NORMAL | cv2.WINDOW_FREERATIO)
        cv2.setWindowProperty(self._win_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

        self._reset_frame()

    def _reset_frame(self):
        frame = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)
        for button in CONTROL_BUTTONS:
            button.draw(frame, False)
        cv2.imshow(self._win_name, frame)

    def gaze_cb(self, msg):
        joy = sensor_msgs.msg.Joy()
        joy.axes = [0., 0.]
        try:
            x = msg.world_data[0].position.x
            y = msg.world_data[0].position.y
        except IndexError: # no data
            self._reset_frame()
        else:
            frame = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)
            for button in CONTROL_BUTTONS:
                if button.check((int(x), int(y))):
                    button.update(joy)
                    button.draw(frame, True)
                else:
                    button.draw(frame, False)
            cv2.imshow(self._win_name, frame)
        self._pub.publish(joy)

def main():
    rospy.init_node("gaze_to_joy")
    g = GazeToJoy()
    rospy.spin()

if __name__ == "__main__":
    main()
    