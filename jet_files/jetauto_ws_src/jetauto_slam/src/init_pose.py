#!/usr/bin/env python3
# encoding: utf-8
# @Author: Aiden
# @Date: 2022/11/18
import math
import rospy
from hiwonder_servo_msgs.msg import CommandDuration, JointState

class InitPose:
    def __init__(self):
        rospy.init_node('init_pose')

        jointp_angle = rospy.get_param('~p_joint',    0)     # JHC: add pan 
        jointt_angle = rospy.get_param('~t_joint',    0)     # JHC: add tilt
        joint1_angle = rospy.get_param('~joint1',   -70)     # degs
        joint2_angle = rospy.get_param('~joint2',  -110)     # degs
        joint3_angle = rospy.get_param('~joint3',   110)     # degs
        joint4_angle = rospy.get_param('~joint4',    90)     # degs
        jointr_angle = rospy.get_param('~r_joint',    0)     # degs
        horizontal = rospy.get_param('~horizontal', False)
        namespace = rospy.get_namespace()

        # JHC: add neck pan and tilt
        jointp = rospy.Publisher('p_joint_controller/command_duration', CommandDuration, queue_size=1)   
        jointt = rospy.Publisher('t_joint_controller/command_duration', CommandDuration, queue_size=1)  
        joint1 = rospy.Publisher( 'joint1_controller/command_duration', CommandDuration, queue_size=1)
        joint2 = rospy.Publisher( 'joint2_controller/command_duration', CommandDuration, queue_size=1)
        joint3 = rospy.Publisher( 'joint3_controller/command_duration', CommandDuration, queue_size=1)
        joint4 = rospy.Publisher( 'joint4_controller/command_duration', CommandDuration, queue_size=1)
        jointr = rospy.Publisher('r_joint_controller/command_duration', CommandDuration, queue_size=1)

        while not rospy.is_shutdown():
            try:
                if rospy.get_param(namespace + 'hiwonder_servo_manager/running') and rospy.get_param(namespace + 'joint_states_publisher/running'):
                    break
            except:
                rospy.sleep(0.1)
        if horizontal:
            joint1.publish(CommandDuration(data=0, duration=2000))
            joint2.publish(CommandDuration(data=math.radians(-59), duration=2000))
            joint3.publish(CommandDuration(data=math.radians(117), duration=2000))
            joint4.publish(CommandDuration(data=math.radians(31), duration=2000))
            jointr.publish(CommandDuration(data=0, duration=2000))
        else:
            # JHC: add neck pan and tilt
            jointp.publish(CommandDuration(data=math.radians(jointp_angle), duration=2000))  
            jointt.publish(CommandDuration(data=math.radians(jointt_angle), duration=2000))  
            joint1.publish(CommandDuration(data=math.radians(joint1_angle), duration=2000))
            joint2.publish(CommandDuration(data=math.radians(joint2_angle), duration=2000))
            joint3.publish(CommandDuration(data=math.radians(joint3_angle), duration=2000))
            joint4.publish(CommandDuration(data=math.radians(joint4_angle), duration=2000))
            jointr.publish(CommandDuration(data=math.radians(jointr_angle), duration=2000))

if __name__ == '__main__':
    InitPose()
