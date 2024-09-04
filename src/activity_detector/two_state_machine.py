#!/usr/bin/env python3
import rospy 
from std_msgs.msg import Header

class ActivityEvent:
    def __init__(self, name: str):
        self.name = name
        self.happened = False
        self._detector = None
        self._start_time = None
        self._stop_time = None
        self._start_seq = None
        self._stop_seq = None
        #these are start to start times.
        self._last_time_duration = None ## Im am going to track full cycle here, but i could also track positive and negative parts
        self._last_seq_duration = None
        self._header_now = Header
    def update_state(self, header_now: Header):
        self._header_now = header_now
        #rospy.logwarn(f"setting local copy of header to :{header_now}")
    def set(self):
        if self._start_time:
            self._last_time_duration = self._header_now.stamp - self._start_time
        if self._start_seq:
            self._last_seq_duration = self._header_now.seq - self._start_seq
        self._start_time = self._header_now.stamp
        self._start_seq = self._header_now.seq
        #rospy.logwarn(f"setting _start_time {self._header_now.stamp} and _start_seq {self._header_now.seq}")
        self.happened = True
    def reset(self):
        self._stop_time = self._header_now.stamp
        self._stop_seq = self._header_now.seq
        self.happened = False
    def set_detector(self, detector):
        self._detector = detector
    def detect(self, state_msg):
        return self._detector(state_msg)
    def get_activity_time_counter(self):
        if self._start_time and self._start_seq:
            return self._header_now.stamp - self._start_time, self._header_now.seq - self._start_seq
        #rospy.logwarn("either _start_time or _start_seq not set")
        return None, None
    def get_percentage_counter(self):
        if self._last_time_duration and abs(self._last_time_duration) > rospy.Duration(0, 1):
            return ((self._header_now.stamp - self._start_time)/self._last_time_duration)%2
        return 0.0

class TwoEventStateMachine:
    def __init__(self, name: str):
        self.name = name
        self._activity1 = None
        self._activity2 = None
        self._current_state = "Unknown"

    def set_activity1(self, activity: ActivityEvent):
        self._activity1 = activity
    def set_activity2(self, activity: ActivityEvent):
        self._activity2 = activity
    def set_state(self, state_msg): ## this state defining msg has to have a header though
        self._activity1.update_state(state_msg.header)
        self._activity2.update_state(state_msg.header)
        if not self._activity1.happened and self._activity1.detect(state_msg):
            #rospy.logerr("="*10+"ACTIVITY1"+"="*10)
            self._activity1.set()
            self._activity2.reset()
        if not self._activity2.happened and self._activity2.detect(state_msg):
            #rospy.logerr("="*10+"ACTIVITY2"+"="*10)
            self._activity2.set()
            self._activity1.reset()
    
    def get_current_state(self):
        if self._activity1.happened:
            self._current_state = self._activity1.name
            return self._current_state, self._activity1.get_activity_time_counter()
        if self._activity2.happened:
            self._current_state = self._activity2.name
            return self._current_state, self._activity2.get_activity_time_counter()
        self._current_state = "Unknown"
        return self._current_state, (None, None)
    def get_percent(self):
        return self._activity1.get_percentage_counter()



