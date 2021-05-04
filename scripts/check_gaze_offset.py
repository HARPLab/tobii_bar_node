#!/usr/bin/env python

import cv2
import numpy as np
import rospy

import ibmmpy.msg

TARGET_THRESHOLD = 0.15 # frac of screen size to say we're looking at the point
TARGET_NUM_SAMPLES = 180   # num of consecutive static points to id as target
TARGET_VARIANCE = 0.015 # allowed variance within on-target points

def get_point_from_norm(pt, frame):
    return (int(pt[0] * frame.shape[1]), int(pt[1] * frame.shape[0]))

class DataCollector:
    def __init__(self, pt, debug=False):
        self._pt = np.array(pt)
        self._started = None
        self._finished = False
        self._data = np.empty((0, 2))
        self._debug = debug

    def is_finished(self):
        return self._finished

    def receive_observation(self, pts):
        if self._finished:
            return

        if not self._started:
            # see if any of the points are within the start time
            dists = np.linalg.norm(pts - self._pt, axis=1)

            outside_pts = np.flatnonzero(dists > TARGET_THRESHOLD)
            if outside_pts.size == 0:
                first_ok_pt = 0
            elif outside_pts[-1] == pts.shape[0]-1:
                return
            else:
                first_ok_pt = outside_pts[-1]+1

            self._data = pts[first_ok_pt:,:]
            self._started = True
            rospy.loginfo("({:.2f}, {:.2f}): Started receiving (dist={dist:.3f})".format(*self._pt, dist=dists[first_ok_pt]))

        else:
            # make sure we are maintaining the required threshold
            last_ok_idx = -1
            for idx in range(pts.shape[0]):
                # premature optimization is the root of all evil
                if np.any(np.std(np.vstack((self._data, pts[:idx+1, :])), axis=0) > TARGET_VARIANCE):
                    last_ok_idx = idx
            if last_ok_idx == -1:
                # all the points are ok
                self._data = np.vstack((self._data, pts))
            else:
                # some point is not ok
                self._data = np.vstack((self._data, pts[:last_ok_idx,:]))
                # we're done collecting data for now
                self._started = False

            if self._data.shape[0] >= TARGET_NUM_SAMPLES:
                self._started = True
                self._finished = True
                rospy.loginfo("({:.2f}, {:.2f}): Finished".format(*self._pt))
            elif not self._started:
                # we broke the collection, so restart
                self._data = np.empty((0,2))
                rospy.loginfo("({:.2f}, {:.2f}): Data broken, clearing".format(*self._pt))

    def get_data(self):
        if self._finished:
            return self._data
        else:
            return None

    def draw_not_started(self, frame):
        cv2.circle(frame, get_point_from_norm(self._pt, frame), 10, (255, 0, 0), -1)
        if self._debug:
            cv2.ellipse(frame, get_point_from_norm(self._pt, frame),
                get_point_from_norm((TARGET_THRESHOLD, TARGET_THRESHOLD), frame),
                0, 0., 360., (255, 255, 255), 1)

    def draw_running(self, frame):
        progress = self._data.shape[0]/float(TARGET_NUM_SAMPLES)
        # animation progress 0-1
        # if progress <= 0.9:
        #     ax1 = int(10 + 5*np.sin(progress*2*np.pi/0.2))
        #     ax2 = int(10 - 5*np.sin(progress*2*np.pi/0.2))
        # else:
        #     ax1 = 15
        #     ax2 = int(50*(1. - progress))
        cv2.circle(frame, get_point_from_norm(self._pt, frame), 10, (255, 0, 0), -1)
        cv2.ellipse(frame, get_point_from_norm(self._pt, frame), (13, 13), 0, 0., 360.*progress, (0, 0, 255), 2)

        if self._debug:
            center_pt = np.mean(self._data, axis=0)
            for p in self._data:
                cv2.circle(frame, get_point_from_norm(p, frame), 1, (0, 255, 255), -1)
            cv2.circle(frame, get_point_from_norm(center_pt, frame), 5, (0, 255, 255), -1)
            cv2.ellipse(frame, get_point_from_norm(center_pt, frame), 
                get_point_from_norm((TARGET_VARIANCE, TARGET_VARIANCE), frame),
                0, 0., 360., (255, 255, 255), 1)

    def draw(self, frame):
        if self._finished:
            return
        elif self._started:
            self.draw_running(frame)
        else:
            self.draw_not_started(frame)

    def draw_results(self, frame):
        cv2.circle(frame, get_point_from_norm(self._pt, frame), 10, (255, 0, 0), -1)
        for r in self._data:
            cv2.circle(frame, get_point_from_norm(r, frame), 1, (0, 255, 255), -1)
        cv2.circle(frame, get_point_from_norm(np.mean(self._data, axis=0), frame), 10, (0, 0, 255), 2)

    def get_offset(self):
        return np.mean(self._data, axis=0) - self._pt

NO_GAZE_WARNING_TIME = rospy.Duration.from_sec(1.)

class CalibrationRunner:
    def __init__(self, points, gaze_source_factory, image_sink, finished_callback, debug=False):
        self._collectors = [ DataCollector(pt, debug=debug) for pt in points ]
        self._gaze_source = gaze_source_factory(self._receive_gaze)
        self._image_sink = image_sink
        self._finished_callback = finished_callback

        self._last_data_time = None
        self._display_timer = rospy.Timer(rospy.Duration.from_sec(0.033), self._display, oneshot=False)
        self._debug = debug
        self._last_gaze_msg = None
        self._finished = False

    def _receive_gaze(self, msg):
        if len(msg.world_data) > 0:
            self._last_data_time = msg.header.stamp
            data = np.array( [ [ p.position.x, p.position.y ] for p in msg.world_data ] )
            self._last_gaze_msg = data[-1,:].copy()
            for c in self._collectors:
                c.receive_observation(data)

            if all((c.is_finished() for c in self._collectors)):
                self._finished = True

    def _display(self, evt):
        if not self._finished:
            # check staleness
            if self._last_data_time is not None and (evt.current_real - self._last_data_time) > NO_GAZE_WARNING_TIME:
                rospy.logwarn_throttle(1., "No gaze data received for {:.03f} sec".format((evt.current_real - self._last_data_time).to_sec()))
            frame = self._image_sink.get_empty()
            if self._debug and self._last_gaze_msg is not None:
                cv2.circle(frame, get_point_from_norm(self._last_gaze_msg, frame), 8, (0, 255, 255), -1)
            for c in self._collectors:
                c.draw(frame)
            self._image_sink.draw(frame)
        else:
            self._display_timer.shutdown()
            self._finished_callback(self._collectors, self._image_sink)

def make_gaze_source_factory(topic):
    def make_gaze_source(cb):
        return rospy.Subscriber(topic, ibmmpy.msg.GazeData, cb, queue_size=1)
    return make_gaze_source

class ImageDisplay:
    WINDOW_NAME = "gaze_checker"
    def __init__(self):
        self._initialized = False

    def get_empty(self):
        return np.zeros((720, 1280, 3), dtype=np.uint8)

    def draw(self, frame):
        if not self._initialized:
            # have to initialize window in *same* thread as displaying so can't do it in __init__
            cv2.namedWindow(ImageDisplay.WINDOW_NAME, cv2.WINDOW_GUI_NORMAL | cv2.WINDOW_FREERATIO)
            cv2.setWindowProperty(ImageDisplay.WINDOW_NAME, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
            self._initialized = True

        cv2.imshow(ImageDisplay.WINDOW_NAME, frame)
        cv2.waitKey(1)

def display_results(collectors, image_sink):
    frame = image_sink.get_empty()
    for c in collectors:
        c.draw_results(frame)
        print("{}: ({:.04f}, {:.04f})".format(tuple(c._pt), *c.get_offset()))

    offsets = [c.get_offset() for c in collectors]
    print("Total offset: ({:.4f}, {:.4f})".format(*np.mean(offsets, axis=0)))

    image_sink.draw(frame)

DEFAULT_CHECK_POINTS = [
    [ 0.1, 0.1 ],
    [ 0.9, 0.1 ],
    [ 0.9, 0.8 ],
    [ 0.1, 0.8 ],
    [ 0.3, 0.5 ],
    [ 0.7, 0.5 ]]

def main():
    rospy.init_node("check_gaze_offset", anonymous=True)
    gaze_topic = rospy.get_param("~gaze_topic", "/gaze_data")
    points = rospy.get_param("~points", DEFAULT_CHECK_POINTS)
    debug = rospy.get_param("~debug", False)
    print(debug)
    if rospy.get_param("publish_image", False):
        raise NotImplementedError()
    else:
        image_sink = ImageDisplay()

    runner = CalibrationRunner(points, make_gaze_source_factory(gaze_topic), image_sink, display_results, debug=debug)

    rospy.spin()

if __name__ == "__main__":
    main()



