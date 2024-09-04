#!/usr/bin/env python3
import rospy
from insole_msgs.msg import InsoleSensorStamped
from opensimrt_msgs.msg import Common, MultiMessage, OpenSimData
from step_detector import StepDetectorMulti

rospy.init_node("walking_detector_node", anonymous=True)

side = rospy.get_param("~side")
sd =  StepDetectorMulti(side)
percent_pub = rospy.Publisher(side+"/percent", MultiMessage, queue_size=10 )

shared_info = None

def callback_y_data(msg: MultiMessage) -> None:

    ##Idk, we are going to deal with synchronization, because plotjuggler doesnt do that
    shared_info = msg

def callback(msg : MultiMessage) -> None:
    
    #Sensor Positions
    # note that numbering starts at 1
    ##
    sd.set_state(msg)
    state_name, (time_duration, seq_duration) = sd.get_current_state()
    #rospy.loginfo(f"\nstate: {state_name},\ntime_duration_of_state: {time_duration},\nseq_duration_of_state: {seq_duration}")
    percentage = sd.get_percent()
    msg_ = MultiMessage()
    msg_.header = msg.header
    msg_.header.frame_id = state_name

    #I have no idea what to do now

    percent_message = OpenSimData()
    percent_message.data = [percentage]
    msg_.ik = msg.ik
    msg_.other = msg.other
    msg_.other.append(percent_message)

    percent_pub.publish(msg_)

rospy.Subscriber("/id_node/output_multi", MultiMessage, callback=callback, queue_size=10)

## some other thing..
#rospy.Subscriber(side+"/insole", InsoleSensorStamped, callback=callback_y_data, queue_size=10)
while not rospy.is_shutdown():
    rospy.spin()
