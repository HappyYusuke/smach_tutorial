#!/usr/bin/env python
# -*- coding: utf-8 -*-
#------------------------------------------------------------------
# Title: smachの練習ノード2
# Author: Yusuke Kanazawa
# Data: 2021/9/21
# Memo: 「hello」と「who」の後「TALK」を３回繰り返して、byeからsucceededで終わる
#------------------------------------------------------------------

import rospy
import smach
import smach_ros



class say_hello(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['hello'])

    def execute(self, userdata):
        rospy.loginfo('HELLO!')
        rospy.sleep(1.0)
        return 'hello'



class say_who(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['who'])

    def execute(self, userdata):
        rospy.loginfo('WHO?')
        rospy.sleep(1.0)
        return 'who'



class Im_hoge(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['hoge1', 'bye'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('I AM HOGE')
        if self.counter < 3:
            self.counter += 1
            rospy.sleep(1.0)
            return 'hoge1'
        else:
            return 'bye'



class say_hoge(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['hoge2'])

    def execute(self, userdata):
        rospy.loginfo('HOGE')
        rospy.sleep(1.0)
        return 'hoge2'

def main():
    # 状態機械を定義
    sm_top = smach.StateMachine(outcomes=['succeeded'])

    # 出力結果と遷移先を定義
    with sm_top:
        smach.StateMachine.add('Say_HELLO', say_hello(), 
                                transitions={'hello':'Say_WHO'})
        
        smach.StateMachine.add('Say_WHO', say_who(), 
                                transitions={'who':'TALK'})

        sm_talk = smach.StateMachine(outcomes=['GOOD_BYE'])

        with sm_talk:
            smach.StateMachine.add('Im_HOGE', Im_hoge(), 
                                    transitions={'hoge1':'HOGE', 'bye':'GOOD_BYE'})

            smach.StateMachine.add('HOGE', say_hoge(),
                                    transitions={'hoge2':'Im_HOGE'})
        
        smach.StateMachine.add('TALK', sm_talk,
                                transitions={'GOOD_BYE':'succeeded'})


    # smach viewerで見えるようにする
    sis = smach_ros.IntrospectionServer('server_name', sm_top, '/SM_ROOT')
    sis.start()

    # 状態機械実行
    outcome = sm_top.execute()
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('smach_sample1')
    main()
