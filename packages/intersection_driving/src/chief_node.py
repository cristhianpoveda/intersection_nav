#! /usr/bin/env python3

import time
import rospy
import actionlib
from duckietown.dtros import DTROS, NodeType
import intersection_msgs.msg
from intersection_msgs.srv import DetectStopSign, DetectStopSignResponse, MakeDecision, MakeDecisionRequest
from duckietown_msgs.srv import SetValue, SetValueRequest
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseResult, MoveBaseFeedback
from duckietown_msgs.msg import FSMState
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
from geometry_msgs.msg import Pose

class ChiefNode(DTROS):
    # create messages that are used to publish feedback/result
    _feedback = intersection_msgs.msg.IntersectionDrivingFeedback()
    _result = intersection_msgs.msg.IntersectionDrivingResult()

    def __init__(self, node_name):
        super(ChiefNode, self).__init__(node_name=node_name, node_type=NodeType.BEHAVIOR)

        # params
        self._way1 = rospy.get_param('~way1')
        self._way2 = rospy.get_param('~way2')
        self._way3 = rospy.get_param('~way3')
        
        self.pub_state = rospy.Publisher('/duckiebot4/fsm_node/mode', FSMState, queue_size=1)

        self._action_name = node_name
        self._as = actionlib.SimpleActionServer(self._action_name, intersection_msgs.msg.IntersectionDrivingAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

    def feedback_callback(self, feedback):
        rospy.loginfo('[Feedback] Going to goal pose')

    def call_swicth_srv(self, srv_name, state):

        switch_response = SetBoolResponse()

        rospy.wait_for_service(srv_name)
        try:
            switch_request = SetBoolRequest()
            switch_request.data = state
            switch_client = rospy.ServiceProxy(srv_name, SetBool)
            switch_response = switch_client(switch_request)

        except rospy.ServiceException as e:
            rospy.loginfo("Failed to switch [%s] on: [%s]"% (srv_name, e))
            switch_response.success = False

        return switch_response

    def fail_msg(self):
        self._result.result = "Intersection driving failed"
        rospy.loginfo('%s: Failed' % self._action_name)
        self._as.set_succeeded(self._result)

    def execute_cb(self, goal):

        success = False
        
        # starting message
        rospy.loginfo('%s: Executing action: Navigate to destination: %s .' % (self._action_name, goal.destination.data))

        # SWITCH STOP SIGN DETECTION ON

        switch_response = self.call_swicth_srv('/duckiebot4/stop_sign_detector_node/switch', True)

        if not switch_response.success:
            self.fail_msg()
            return

        # ARRIVE AT STOP SIGN

        rospy.wait_for_service('/duckiebot4/stop_sign_detector_node/detect_stop_sign')
        try:
            arrived_response = Pose()
            arr = rospy.ServiceProxy('/duckiebot4/stop_sign_detector_node/detect_stop_sign', DetectStopSign)
            arrived_response = arr()

            # 1st task feedback
            self._feedback.tasks = "Arrived at intersection"
            self._as.publish_feedback(self._feedback)

        except rospy.ServiceException as e:
            rospy.loginfo("Arrive at stop sign service call failed: %s"%e)
            self.fail_msg()
            return

        # SWITCH STOP SIGN DETECTION OFF

        switch_response = self.call_swicth_srv('/duckiebot4/stop_sign_detector_node/switch', False)

        if not switch_response.success:
            self.fail_msg()
            return

        # SWITCH OBJECT DETECTION ON

        switch_response = self.call_swicth_srv('/duckiebot4/runtime_detector/switch', True)

        if not switch_response.success:
            self.fail_msg()
            return

        # UPDATE DUCKIEBOT POSE ON MAP

        rospy.wait_for_service('/duckiebot4/velocity_to_pose_node/update_pose')
        try:
            actual_pose = SetValueRequest()
            actual_pose.value = - 0.2
            pose = rospy.ServiceProxy('/duckiebot4/velocity_to_pose_node/update_pose', SetValue)
            pose()

            # 2nd task feedback
            self._feedback.tasks = "Duckiebot located on intersection map"
            self._as.publish_feedback(self._feedback)

        except rospy.ServiceException as e:
            rospy.loginfo("Set Pose service call failed: %s"%e)
            self.fail_msg()
            return

        time.sleep(0.13)

        # MAKE DECISION
        decision = 1

        rospy.wait_for_service('/srv_decision')
        try:
            destination_request = MakeDecisionRequest()
            destination_request.destination.data = goal.destination.data
            destination_request.stop_dist.data = 0.2
            inf = rospy.ServiceProxy('/srv_decision', MakeDecision)
            decision_resp = inf(destination_request)

            # 3rd task feedback
            self._feedback.tasks = decision_resp.decision.data
            self._as.publish_feedback(self._feedback)

            decision = decision_resp.decision.data

        except rospy.ServiceException as e:
            rospy.loginfo("Decision making service call failed: %s"%e)
            self.fail_msg()
            return

        # SWITCH OBJECT DETECTION OFF

        switch_response = self.call_swicth_srv('/duckiebot4/runtime_detector/switch', False)

        if not switch_response.success:
            self.fail_msg()
            return

        # SWITCH LOCALIZATION ON

        switch_response = self.call_swicth_srv('/duckiebot4/forward_kinematics_node/switch', True)

        if not switch_response.success:
            self.fail_msg()
            return

        # CHANGE FINITE STATE MACHINE TO INTERSECTION CONTROL

        state_nav = FSMState()
        state_nav.state = "INTERSECTION_CONTROL"
        self.pub_state.publish(state_nav)

        success = True

        # NAVIGATE

        if decision == 1:

            # Decision feedback
            self._feedback.tasks = "Action: wait"
            self._as.publish_feedback(self._feedback)
            success = True

        else:
            # Decision feedback
            self._feedback.tasks = "Aciton: cross"
            self._as.publish_feedback(self._feedback)

            # SET NAVIGATION GOAL

            if goal.destination.data == "1":
                x = self._way1.x
                y = self._way1.r
                rz = self._way1.rz
                rw = self._way1.rw

            elif goal.destination.data == "2":
                x = self._way2.x
                y = self._way2.r
                rz = self._way2.rz
                rw = self._way2.rw

            elif goal.destination.data == "3":
                x = self._way3.x
                y = self._way3.r
                rz = self._way3.rz
                rw = self._way3.rw

            nav_goal = MoveBaseGoal()
            nav_goal.target_pose.header.frame_id = 'map'
            nav_goal.target_pose.pose.position.x = x
            nav_goal.target_pose.pose.position.y = y
            nav_goal.target_pose.pose.position.z = 0.0
            nav_goal.target_pose.pose.orientation.x = 0.0
            nav_goal.target_pose.pose.orientation.y = 0.0
            nav_goal.target_pose.pose.orientation.z = rz
            nav_goal.target_pose.pose.orientation.w = rw

            client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)

            client.wait_for_server()

            client.send_goal(nav_goal, feedback_cb = self.feedback_callback)

            client.wait_for_result()

            rospy.loginfo('[Result] State: %d'% (client.get_state()))

            success = True

        # SWITCH LOCALIZATION OFF

        switch_response = self.call_swicth_srv('/duckiebot4/forward_kinematics_node/switch', False)

        if not switch_response.success:
            self.fail_msg()
            return

        state_end = FSMState()
        state_end.state = "NORMAL_JOYSTICK_CONTROL"
        self.pub_state.publish(state_end)
          
        if success:
            self._result.result = "Intersection driving finished"
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)
        
if __name__ == '__main__':
    node = ChiefNode(node_name='chief_node')
    rospy.spin()