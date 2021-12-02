#!/usr/bin/env python
import rospy
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander
from intera_interface import gripper as robot_gripper
import numpy as np
from numpy import linalg
import sys

def main(robo):
    # Wait for the IK service to become available
    rospy.wait_for_service('compute_ik')
    rospy.init_node('service_query')
    arm = 'left'
    # Create the function used to call the service
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
    if robo == 'sawyer':
    	arm = 'right'
    right_gripper = robot_gripper.Gripper(arm)
    right_gripper.calibrate()
    while not rospy.is_shutdown():
        raw_input('Press [ Enter ]: ')
        
        # Construct the request
        request_go_grab = GetPositionIKRequest()
        request_go_grab.ik_request.group_name = arm + "_arm"

        # If a Sawyer does not have a gripper, replace '_gripper_tip' with '_wrist' instead
        link = arm + "_gripper"
        if robo == 'sawyer':
        	link += '_tip'

        request_go_grab.ik_request.ik_link_name = link
        request_go_grab.ik_request.attempts = 20
        request_go_grab.ik_request.pose_stamped.header.frame_id = "base"
        
        # Set the desired orientation for the end effector HERE
        request_go_grab.ik_request.pose_stamped.pose.position.x = 0.700
        request_go_grab.ik_request.pose_stamped.pose.position.y = -0.050
        request_go_grab.ik_request.pose_stamped.pose.position.z =  -0.099        
        request_go_grab.ik_request.pose_stamped.pose.orientation.x = 0.0
        request_go_grab.ik_request.pose_stamped.pose.orientation.y = 1.0
        request_go_grab.ik_request.pose_stamped.pose.orientation.z = 0.0
        request_go_grab.ik_request.pose_stamped.pose.orientation.w = 0.0

        inter_request = GetPositionIKRequest()
        inter_request.ik_request.group_name = arm + "_arm"
        inter_request.ik_request.ik_link_name = link
        inter_request.ik_request.attempts = 20
        inter_request.ik_request.pose_stamped.header.frame_id = "base"

        inter_request.ik_request.pose_stamped.pose.position.x = .722
        inter_request.ik_request.pose_stamped.pose.position.y = -0.106
        inter_request.ik_request.pose_stamped.pose.position.z =  .251       
        inter_request.ik_request.pose_stamped.pose.orientation.x = 0.0
        inter_request.ik_request.pose_stamped.pose.orientation.y = 1.0
        inter_request.ik_request.pose_stamped.pose.orientation.z = 0.0
        inter_request.ik_request.pose_stamped.pose.orientation.w = 0.0

        # final_request = GetPositionRequest()
        # final_request.ik_request.group_name = arm + "_arm"
        # final_request.ik_request.ik_link_name = link
        # final_request.ik_request.attempts = 20
        # final_request.ik_request.pose_stamped.header.frame_id = "base"

        # final_request.ik_request.pose_stamped.pose.position.x = .722
        # final_request.ik_request.pose_stamped.pose.position.y = -0.106
        # final_request.ik_request.pose_stamped.pose.position.z =  .251       
        # final_request.ik_request.pose_stamped.pose.orientation.x = 0.0
        # final_request.ik_request.pose_stamped.pose.orientation.y = 1.0
        # final_request.ik_request.pose_stamped.pose.orientation.z = 0.0
        # final_request.ik_request.pose_stamped.pose.orientation.w = 0.0
        
        try:
            right_gripper.open()
            rospy.sleep(1.0)
            # Send the request to the service
            response = compute_ik(request_go_grab)
            
            # Print the response HERE
            print(response)
            group = MoveGroupCommander(arm + "_arm")

            # Setting position and orientation target
            group.set_pose_target(request_go_grab.ik_request.pose_stamped)

            # TRY THIS
            # Setting just the position without specifying the orientation
            #group.set_position_target([0.5, 0.5, 0.0])

            # Plan IK and execute
            group.go()
            right_gripper.close()
            rospy.sleep(1.0)

            response = compute_ik(inter_request)
            group = MoveGroupCommander(arm + "_arm")
            group.set_pose_target(inter_request.ik_request.pose_stamped)
            group.go()
            rospy.sleep(1.0)

            right_gripper.open()
            rospy.sleep(1.0)
            
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

# Python's syntax for a main() method
if __name__ == '__main__':
    main(sys.argv[1])

