#!/usr/bin/env python3
import rospy
from insole_msgs.msg import InsoleSensorStamped
from opensimrt_msgs.msg import MultiMessage, OpenSimData
from geometry_msgs.msg import WrenchStamped, Vector3
from two_state_machine import ActivityEvent, TwoEventStateMachine
    
    #Sensor Positions
    # note that numbering starts at 1
    ##
"""
            MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM
            MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM
            MMMMMMMMMMMMMMMMMMMMMMMMMMMgMMMMFFMMMMMMg@MMMMMMMMMMMMMMMMMMMMMMMMMMgMMMMMF77MMMMg@MMMMMMMMMMMMMMMMMMMMMMMMMM
            MMMMMMMMMMMMMMMMMMMMMMMQ@MF```````MM`````MMgMMMMMMMMMMMMMMMMMMMMMgMM`````M````````MMM@MMMMMMMMMMMMMMMMMMMMMMM
            MMMMMMMMMMMMMMMMMMMMMgMMM`````````MM```````MM@MMMMMMMMMMMMMMMMMQMF```````M``````````MMMgMMMMMMMMMMMMMMMMMMMMM
            MMMMMMMMMMMMMMMMMMMQMF`MM`````````MM`````````MMMMMMMMMMMMMMMMMMM`````````M``````````M@`MM@MMMMMMMMMMMMMMMMMMM
            MMMMMMMMMMMMMMMMMM@M```MM```ggg_``MM``ggMMMgg`MMMMMMMMMMMMMMMMM``````````M``````````M@```MMMMMMMMMMMMMMMMMMMM
            MMMMMMMMMMMMMMMMMMg_```MM`gMMMMMg`MM``M 14 MM`MMMMMMMMMMMMMMMMM``````````M``````````M@````MM@MMMMMMMMMMMMMMMM
            MMMMMMMMMMMMMMMMMMMMMg`MM`M 15 MMMMM``MM@MMMM``MMMMMMMMMMMMMMM```````````M``````````M@``````MMMMMMMMMMMMMMMMM
            MMMMMMMMMMMMMMgM 16 MM`MM`7MMMMMF`MM````7MF````MMMMMMMMMMMMMMM```````````M``````````M@```````MgMMMMMMMMMMMMMM
            MMMMMMMMMMMMM@MMMMMMM``MM`````````gMggggggggggggMMMMMMMMMMMMMggggggggggggMg`````````M@````````M@MMMMMMMMMMMMM
            MMMMMMMMMMMM@M``````_ggMMg@MMMMMMMMFFMM`````````MMMMMMMMMMMMM`````````MM77MMMMMMMM@g@@gg_``````M@MMMMMMMMMMMM
            MMMMMMMMMMM@ggggMMMMMMM``````M@``````MM`````````M@MMMMMMMMM@M`````````@M``````MM``````MMMMMMMgggg@MMMMMMMMMMM
            MMMMMMMMMM@MFMM``````M@``````M@``````MM`````````M@MMMMMMMMMMM`````````@M``````MM``````MM``````MM7MgMMMMMMMMMM
            MMMMMMMMMgM``MM``````M@``````M@``````MM`````````M@MMMMMMMM@MM`````````@M``````MM``````MM``````MM``MMMMMMMMMMM
            MMMMMMMMMM```MM``````M@``````M@`gggg_MM`g@MMMMg`M@MMMMMMMM@MM`````````@M``````MM``````MM``````MM```MMMMMMMMMM
            MMMMMMMMMM```MM``__``M@g@MMMgM@MMMMMMMM`MMg 9 MFM@MMMMMMMMMMM`````````@M``````MM``````MM``````MM```M@MMMMMMMM
            MMMMMMMgM__``MMgMMMM@MM@ 11 MMMM 10 MMM`MMMMMMM`@MMMMMMMMMM@@`````````@M``````MM``````MM``````MM````MMMMMMMMM
            MMMMMMMMMMMMgMMM 12 MMMM@MMMMM@7MMMMFMM````F````MMMMMMMMMMMMM`````````@M``````MM``````MM``````MM````MMMMMMMMM
            MMMMMMMM 13 MMMMMMMMMM@``FFF`M@``````MM````````MMgMMMMMMMMMgMM````````@M``````MM``````MM``````MM`````MMMMMMMM
            MMMMMMMMMMMMMMM``````M@``````M@``````MM````````@MMMMMMMMMMMM@g````````@M``````MM``````MM``````MM`````M@MMMMMM
            MMMMMMM``````MM``````M@``````M@``````MM```````MMgMMMMMMMMMMMgMg```````@M``````MM``````MM``````MM`````MMMMMMMM
            MMMMMMM``````MM``````M@``````M@``````M@```___gMMMMMMMMMMMMMMMMMg___```@M``````MM``````MM``````MM``````MMMMMMM
            MMMMM@@``````MM``````M@`_gggg@MMMMMMMMMMMMMMMMQMMMMMMMMMMMMMMM@MMMMMMMMMMMMMMMM@gggg_`MM``````MM``````MMMMMMM
            MMMMMMM``````MM`ggggMMMMMMMMMMMMMMMMMMM`````gMMMMMMMMMMMMMMMMM@Mg`````@MMMMMMMMMMMMMMMMMMgggg`MM``````M@MMMMM
            MMMM@MM``ggg@MMMMMMMMM X MMMMMMMMMMMMMM`7``gM@MMMMMMMMMMMMMMMMM@Mg````@MMMMMMMMMMMMMM X MMMMMMMM@ggg``M@MMMMM
            MMMM@MMMMMF``MMMMMMMMM   MMMMMMMMMMMMMM```gM@MMMMMMMMMMMMMMMMMMM@Mg```@MMMMMMMMMMMMMM   MMMMMMMM``FMMMMMMMMMM
            MMMMMMM``````MMMMMMMMM A MMMMMMMMMMMMM@@gMM@MMMMMMMMMMMMMMMMMMMMMgMg``@MMMMMMMMMMMMMM A MMMMMMMM``````M@MMMMM
            MMMMM@@``````MMMM    M | MMMMMMMMMMMMM@QMMMMMMMMMMMMMMMMMMMMMMMMMMgMg`MMMMMMMMMMMMMMM | M    MMM``````@MMMMMM
            MMMMMMM``    MMMM CL M | MMMMMMMMMMMMMQMMMMMMMMMMMMMMMMMMMMMMMMMMMMgMgMMMMMMMMMMMMMMM | M CR MMM``````MMMMMMM
            MMMMMMM      MMMM    M O----->  Y MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMgMMMMM Y  <-----O M    MM``````QM@MMMMM
            MMMMMMMM  8  MMMMMMMMM   MMMMMMMMMMMM%MQMMMMMMMMMMMMMMMMMMMMMMMMMMMMM@@MMMMMMMMMMMMMM   MMMMMMM``````@MMMMMMM
            MMMMMMMMM     `MMMMMMM Z MMMMMMMMMMM gMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM@MgMMMMMMMMMMMMM Z MMMMMM```````MMMMMMMM
            MMMMMMMMm```````MMMMMM   MMMMMMMMMM qMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMmMMMMMMMMMMMM   MMMMM```````QMMMMMMMM
            MMMMMM@MM`````````MMMMMMMMMMMMMMMMgg@MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMgggMMMMMMMMMMMMMMMM`````````MMMMMMMMM
            MMMMMMMM@```````ggggMMMMMMMMFFF`````MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM````FF77MMMMMMMMgggg```````@MMMMMMMM
            MMMMMMMMMgggMMMMF```MMMMM``````````MM@MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM@MM``````````MMMMM```7MMMMgggMMMMMMMMM
            MMMMMMMMMF``````````MMMMM```ggg_```@@MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM``````````MMMMM``````````MMMMMMMMMM
            MMMMMMMMMM```gggg```MMMMM`gMMMMM@``MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM@``````````MMMMM``````````qM@MMMMMMM
            MMMMMMM@MM`gMMMMMMr`MMMMM`MM 5 MMM`MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM``````````MMMMM``````````MM@MMMMMMM
            MMMMMMM@MM`MM 6 MMF`MMMMM``MMMMMF``MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM``````````MMMMM``````````M@MMMMMMMM
            MMMMMMMM@M``MMMMMF``MMMMM`````````MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM``````````MMMMM``````````@@MMMMMMMM
            MMMMMMMM@M``````````MMMMM`````````MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM``````````MMMMM``````````@@MMMMMMMM
            MMMMMMMMM@```````__ggMMMM@MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM@@MMMgg__```````@MMMMMMMMM
            MMMMMMMMM&gg@MMMMMFF`MMM``````````MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM```````````MMM`F7MMMMM@gg@MMMMMMMMM
            MMMMMMMMMM```````````MMM```````````MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM```````````MMM```````````MMMMMMMMMM
            MMMMMMMMM@````___````MMM``ggMMMg[``MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM```````````MMM```````````MMMMMMMMMM
            MMMMMMMMM@``gMMMMMg``MMM``MM 3 MM``MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM```````````MMM```````````MMMMMMMMMM
            MMMMMMMMM@`MM 4 MMM``MMM``MMMM@MM``MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM@```````````MMM```````````MMMMMMMMMM
            MMMMMMMM@M``MMMMMMF``MMM````7MF````MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM@@```````````MMM```````````@MMMMMMMMM
            MMMMMMMM@M```````````MMM```````````@@MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM@M```````````MMM```````````@@MMMMMMMM
            MMMMMMMM@M```````````MMg_ggggggggggg@MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM@ggggggggggg_gMM```````````M@MMMMMMMM
            MMMMMMMMMgggggg@MMMMMMMMFFFF```````M@MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM```````FF77MMMMMMMM@gggggg@MMMMMMMM
            MMMMMMMMMMF``````````MM````````````M@MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM```````````MM```````````FM@MMMMMMMM
            MMMMMMMMM@````gggg_``MM``_gMMMgg```@MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM@```````````MM````````````@@MMMMMMMM
            MMMMMMMMMM```MMMMMMg`MM``MM 1 MMM``MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM```````````MM````````````MMMMMMMMMM
            MMMMMMMMgMg`MM 2 MMM`MM``MMM@@MM``MMgMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMgMg``````````MM```````````MMgMMMMMMMM
            MMMMMMMMMMMg`7MMMMM``MM````7FF```qMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMg`````````MM``````````qMMMMMMMMMMM
            MMMMMMMMMMMMg````````MM`````````gMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM@Mg````````MM`````````gMgMMMMMMMMMM
            MMMMMMMMMMM@@Mg_`````MM```````g@MgMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMg``````MM``````_gMQ@MMMMMMMMMMM
            MMMMMMMMMMMMMMgMMggg_MM``_gg@MQ@MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM@Mgggg`MM`_gggMMgMMMMMMMMMMMMMM
            MMMMMMMMMMMMMMMMMMgMMMMMMM@gMMMMMMMMMM Left MMMMMMMMMMMMMMMMMMM Right MMMMMMMMMMMgMMM@MMMMgMMMMMMMMMMMMMMMMMM
            MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM
"""

class StepDetector(TwoEventStateMachine):
    def __init__(self):
        super().__init__("step_detector")
        self.heel_strike = ActivityEvent("heel_strike")
        self.toe_off = ActivityEvent("toe_off")

        self.heel_strike.set_detector(self.heel_strike_detector)
        self.toe_off.set_detector(self.toe_off_detector)

        self.set_activity1(self.heel_strike)
        self.set_activity2(self.toe_off)

    def heel_strike_detector(self, msg: InsoleSensorStamped):
        ## heel_strike_detector
        ## this is a placeholder for some more advanced thing, maybe this needs to be a class to remember past states, idk
        if (msg.pressure.data[0] + msg.pressure.data[1] > 5) and (msg.pressure.data[13] + msg.pressure.data[14] + msg.pressure.data[15] < 7.5):
            return True

    def toe_off_detector(self, msg: InsoleSensorStamped):
        ## toe_off_detector
        if (msg.pressure.data[0] + msg.pressure.data[1] < 5) and (msg.pressure.data[13] + msg.pressure.data[14] + msg.pressure.data[15] > 7.5): ## this is wrong, i need to wait until the tip sensors go to zero to detect this
            ## toe_off
            return True

def vector_from_opensimdata(data: OpenSimData):
    v = Vector3
    ### its point, force, torque
    v.x = data[3+0] # i don't think it will be this easy
    v.y = data[3+1]
    v.z = data[3+2]
    return v
    
def norm(v: Vector3):
    return v.x*v.x+v.y*v.y+v.z*v.z

def close_to_zero(v: Vector3):
    return norm(v) < 100 #idk

class StepDetectorWrench(TwoEventStateMachine):
    def __init__(self):
        super().__init__("step_detector")
        self.heel_strike = ActivityEvent("heel_strike")
        self.toe_off = ActivityEvent("toe_off")

        self.heel_strike.set_detector(self.heel_strike_detector)
        self.toe_off.set_detector(self.toe_off_detector)

        self.set_activity1(self.heel_strike)
        self.set_activity2(self.toe_off)
        self.last_force = Vector3

    def heel_strike_detector(self, msg: WrenchStamped):
        ## heel_strike_detector
        ## this is a placeholder for some more advanced thing, maybe this needs to be a class to remember past states, idk
        if norm(msg.wrench.force) > norm(self.last_force) and close_to_zero(self.last_force):
            self.last_force = msg.wrench.force
            return True
        self.last_force = msg.wrench.force

    def toe_off_detector(self, msg: WrenchStamped):
        ## toe_off_detector
        if norm(msg.wrench.force) < norm(self.last_force) and close_to_zero(msg.wrench.force):
            return True

from copy import deepcopy

class StepDetectorMulti(TwoEventStateMachine):
    def __init__(self, side):
        super().__init__("step_detector")
        self.heel_strike = ActivityEvent("heel_strike")
        self.toe_off = ActivityEvent("toe_off")

        self.heel_strike.set_detector(self.heel_strike_detector)
        self.toe_off.set_detector(self.toe_off_detector)

        self.set_activity1(self.heel_strike)
        self.set_activity2(self.toe_off)
        self.last_force = 100

        self.rising = 0
        self.lowering = 0
        #1 left 2 right
        self.side_index = None
        if side == "right":
            self.side_index = 2
        if side == "left":
            self.side_index = 1


    def heel_strike_detector(self, msg: MultiMessage):
        ## heel_strike_detector
        ## this is a placeholder for some more advanced thing, maybe this needs to be a class to remember past states, idk
        force_v = vector_from_opensimdata(msg.other[self.side_index].data)
        #rospy.logwarn(f"heel detector {force_v.x, force_v.y, force_v.z}")
        this_norm = norm(force_v)
        rospy.logwarn_throttle(1,this_norm)

        if norm(force_v) > self.last_force and not close_to_zero(force_v): # and self.last_force < 10:
            self.last_force = this_norm
            self.rising +=1
            self.lowering = 0
        if this_norm < self.last_force:
            self.rising = 0
        if self.rising > 5:
            self.rising = 0
            rospy.logwarn("heel_strike detected")
            return True
        self.last_force = deepcopy(this_norm)

    def toe_off_detector(self, msg: MultiMessage):
        ## toe_off_detector
        force_v = vector_from_opensimdata(msg.other[self.side_index].data)
        this_norm = norm(force_v)
        #rospy.logwarn(f"toe_off_detector {force_v.x, force_v.y, force_v.z}")
        if this_norm < self.last_force and close_to_zero(force_v):
            self.lowering +=1
            self.rising = 0
        if this_norm > self.last_force:
            self.lowering = 0
        if self.lowering > 5:
            self.lowering = 0
            rospy.logwarn("toe_off detected")
            return True

