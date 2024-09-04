#!/usr/bin/env python3
import rospy
from insole_msgs.msg import InsoleSensorStamped
from opensimrt_msgs.msg import Common
from step_detector import StepDetector

rospy.init_node("walking_detector_node", anonymous=True)

sd =  StepDetector()

side = rospy.get_param("~side")
percent_pub = rospy.Publisher(side+"/percent", Common, queue_size=10 )

shared_number= 0

def callback_y_data(msg: Common) -> None:

    ##Idk, we are going to deal with synchronization, because plotjuggler doesnt do that
    shared_number = msg.data[0]

def callback(msg : InsoleSensorStamped) -> None:
    
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
    sd.set_state(msg)
    state_name, (time_duration, seq_duration) = sd.get_current_state()
    #rospy.loginfo(f"\nstate: {state_name},\ntime_duration_of_state: {time_duration},\nseq_duration_of_state: {seq_duration}")
    percentage = sd.get_percent()
    msg_ = Common()
    msg_.header = msg.header
    msg_.header.frame_id = state_name

    ## here is easy, we are getting everything we need, but in other cases, there will be sync issues
    shared_number = msg.pressure.data[0]
    msg_.data = [percentage, shared_number]
    percent_pub.publish(msg_)

rospy.Subscriber(side+"/insole", InsoleSensorStamped, callback=callback, queue_size=10)

## some other thing..
#rospy.Subscriber(side+"/insole", InsoleSensorStamped, callback=callback_y_data, queue_size=10)
while not rospy.is_shutdown():
    rospy.spin()
