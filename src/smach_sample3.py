#!/usr/bin/env python
# -*- coding: utf-8 -*-
#------------------------------------------------------------------
# Title: smachの練習ノード3
# Author: Yusuke Kanazawa
# Data: 2021/9/21
# Memo: sample1にremappingを使用してみた
#------------------------------------------------------------------

import rospy
import smach
import smach_ros

class say_hello(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['hello', 'bye'],
                                   input_keys=['hello_counter_in'],
                                   output_keys=['hello_counter_out'])

    def execute(self, userdata):
        rospy.loginfo('HELLO!')
        rospy.sleep(1.0)
        if userdata.hello_counter_in < 3:
            userdata.hello_counter_out = userdata.hello_counter_in + 1
            return 'hello'
        else:
            return 'bye'

class say_who(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['who'],
                             input_keys=['who_counter_in']) #ここまでしかやってない（まだ途中）

    def execute(self, userdata):
        rospy.loginfo('WHO?')
        rospy.loginfo('Counter = %f' %userdata.who_counter_in)
        rospy.sleep(1.0)
        return 'who'

def main():
    # 状態機械を定義
    sm = smach.StateMachine(outcomes=['good_bye'])
    sm.userdata.sm_counter = 0

    # 出力結果と遷移先を定義
    with sm:
        smach.StateMachine.add('Say_HELLO', say_hello(), 
                                transitions={'hello':'Say_WHO', 'bye':'good_bye'},
                                remapping={'hello_counter_in':'sm_counter',
                                           'hello_counter_out':'sm_counter'})
        
        smach.StateMachine.add('Say_WHO', say_who(), 
                                transitions={'who':'Say_HELLO'},
                                remapping={'who_counter_in':'sm_counter'})

    # smach viewerで見えるようにする
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # 状態機械実行
    outcome = sm.execute()
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('smach_sample1')
    main()
