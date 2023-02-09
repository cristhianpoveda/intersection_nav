#! /usr/bin/env python3

import rospy
import actionlib
from duckietown.dtros import DTROS, NodeType
import intersection_msgs.msg
from intersection_msgs.srv import DetectStopSign, DetectStopSignResponse, MakeDecision, MakeDecisionResponse
from duckietown_msgs.srv import SetValue, SetValueResponse
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseResult, MoveBaseFeedback
from duckietown_msgs.msg import FSMState
from geometry_msgs.msg import PoseWithCovarianceStamped
from robot_localization.srv import SetPose, SetPoseRequest

class CoordinatorNode(DTROS):
    # create messages that are used to publish feedback/result
    _feedback = intersection_msgs.msg.IntersectionDrivingFeedback()
    _result = intersection_msgs.msg.IntersectionDrivingResult()

    def __init__(self, node_name):
        super(CoordinatorNode, self).__init__(node_name=node_name, node_type=NodeType.BEHAVIOR)

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


      
    def execute_cb(self, goal):
        # helper variables
        success = False
        
        # start information
        rospy.loginfo('%s: Executing action: Navigate to destination: %s .' % (self._action_name, goal.destination))


        # ARRIVE AT STOP SIGN

        rospy.wait_for_service('/duckiebot4/stop_sign_detector_node/detect_stop_sign')
        try:
            arr = rospy.ServiceProxy('/duckiebot4/stop_sign_detector_node/detect_stop_sign', DetectStopSign)
            arrived_response = arr()
        except rospy.ServiceException as e:
            rospy.loginfo("Arrive at stop sign service call failed: %s"%e)
        
        # 1st task feedback
        self._feedback.tasks = "Arrived at intersection"
        self._as.publish_feedback(self._feedback)


        # UPDATE DUCKIEBOT POSE ON MAP

        rospy.wait_for_service('/duckiebot4/velocity_to_pose_node/update_pose')
        try:
            actual_pose = SetPoseRequest()
            actual_pose.pose.header.frame_id = 'odom'
            actual_pose.pose.pose.pose.position.x = arrived_response.distance.data
            actual_pose.pose.pose.pose.orientation.w = 1.0
            pose = rospy.ServiceProxy('/set_pose', SetPose)
            pose()
        except rospy.ServiceException as e:
            rospy.loginfo("Set Pose service call failed: %s"%e)
        
        # 2nd task feedback
        self._feedback.tasks = "Duckiebot located on intersection map"
        self._as.publish_feedback(self._feedback)


        # MAKE DECISION
        decision = 1

        rospy.wait_for_service('/srv_decision')
        try:
            inf = rospy.ServiceProxy('/srv_decision', MakeDecision)
            decision_resp = inf(goal.destination)
        except rospy.ServiceException as e:
            rospy.loginfo("Arrive at stop sign service call failed: %s"%e)
        
        # 3rd task feedback
        self._feedback.tasks = decision_resp.decision.data
        self._as.publish_feedback(self._feedback)

        decision = decision_resp.decision.data

        if decision == 0:

            # Decision feedback
            self._feedback.tasks = "Aciton: cross"
            self._as.publish_feedback(self._feedback)

            state_nav = FSMState()
            state_nav.state = "INTERSECTION_CONTROL"
            self.pub_state.publish(state_nav)

            # SET NAVIGATION GOAL

            if goal.destination == "way1":
                x = self._way1.x
                y = self._way1.r
                rz = self._way1.rz
                rw = self._way1.rw

            elif goal.destination == "way2":
                x = self._way2.x
                y = self._way2.r
                rz = self._way2.rz
                rw = self._way2.rw

            elif goal.destination == "way3":
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

        else:
            # decision feedback
            self._feedback.tasks = "Action: wait"
            self._as.publish_feedback(self._feedback)

        state_end = FSMState()
        state_end.state = "NORMAL_JOYSTICK_CONTROL"
        self.pub_state.publish(state_end)

        success = True
          
        if success:
            self._result.result = "Intersection driving finished"
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)
        
if __name__ == '__main__':
    node = CoordinatorNode(node_name='coordinator_node')
    rospy.spin()