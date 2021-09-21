#!/usr/bin/env python
# -*- coding: utf-8 -*-
#------------------------------------------------------------------
# Title: smachの練習ノード
# Author: Yusuke Kanazawa
# Data: 2021/9/21
# Memo: ３回「hello」と「who」をくりかえして「good bye」で終わる
#------------------------------------------------------------------

import rospy
import smach
import smach_ros

class say_hello(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['hello', 'bye'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('HELLO!')
        rospy.sleep(1.0)
        if self.counter < 3:
            self.counter += 1
            return 'hello'
        else:
            return 'bye'

class say_who(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['who'])

    def execute(self, userdata):
        rospy.loginfo('WHO?')
        rospy.sleep(1.0)
        return 'who'

def main():
    # 状態機械を定義
    sm = smach.StateMachine(outcomes=['good_bye'])

    # 出力結果と遷移先を定義
    with sm:
        smach.StateMachine.add('Say_HELLO', say_hello(), transitions={'hello':'Say_WHO', 'bye':'good_bye'})
        
        smach.StateMachine.add('Say_WHO', say_who(), transitions={'who':'Say_HELLO'})

    # smach viewerで見えるようにする
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # 状態機械実行
    outcome = sm.execute()
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('smach_sample1')
    main()
