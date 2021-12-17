#!/usr/bin/env python
import rospy
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander
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
    while not rospy.is_shutdown():
        #points = [(0.805, -0.078), (0.664, -0.093), (0.786, -0.422), (0.669, -0.538)]
        points = [(0.7597, -0.254)]
        previousx = 0.7597
        previousy = -0.254
        #previousx = 0.805
        #previousy = -0.078
        raw_input('Press [ Enter ]: ')
        for x,y in points:
            ##lift hand
            # Construct the request
            request = GetPositionIKRequest()
            request.ik_request.group_name = arm + "_arm"

            # If a Sawyer does not have a gripper, replace '_gripper_tip' with '_wrist' instead
            link = arm + "_gripper"
            if robo == 'sawyer':
                link += '_tip'

            request.ik_request.ik_link_name = link
            request.ik_request.attempts = 20
            request.ik_request.pose_stamped.header.frame_id = "base"
            
            # Set the desired orientation for the end effector HERE
            request.ik_request.pose_stamped.pose.position.x = previousx
            request.ik_request.pose_stamped.pose.position.y = previousy
            request.ik_request.pose_stamped.pose.position.z = -0.05        
            request.ik_request.pose_stamped.pose.orientation.x = 0.0
            request.ik_request.pose_stamped.pose.orientation.y = 1.0
            request.ik_request.pose_stamped.pose.orientation.z = 0.0
            request.ik_request.pose_stamped.pose.orientation.w = 0.0
            
            try:
                # Send the request to the service
                response = compute_ik(request)
                
                # Print the response HERE
                print(response)
                group = MoveGroupCommander(arm + "_arm")

                # Setting position and orientation target
                group.set_pose_target(request.ik_request.pose_stamped)

                # TRY THIS
                # Setting just the position without specifying the orientation
                #group.set_position_target([0.5, 0.5, 0.0])

                # Plan IK and execute
                group.go()
                
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e

            ##execute
            # Construct the request
            previousx = x 
            previousy = y
            request = GetPositionIKRequest()
            request.ik_request.group_name = arm + "_arm"

            # If a Sawyer does not have a gripper, replace '_gripper_tip' with '_wrist' instead
            link = arm + "_gripper"
            if robo == 'sawyer':
            	link += '_tip'

            request.ik_request.ik_link_name = link
            request.ik_request.attempts = 20
            request.ik_request.pose_stamped.header.frame_id = "base"
            
            # Set the desired orientation for the end effector HERE
            request.ik_request.pose_stamped.pose.position.x = x
            request.ik_request.pose_stamped.pose.position.y = y
            request.ik_request.pose_stamped.pose.position.z = -0.05        
            request.ik_request.pose_stamped.pose.orientation.x = 0.0
            request.ik_request.pose_stamped.pose.orientation.y = 1.0
            request.ik_request.pose_stamped.pose.orientation.z = 0.0
            request.ik_request.pose_stamped.pose.orientation.w = 0.0
            
            try:
                # Send the request to the service
                response = compute_ik(request)
                
                # Print the response HERE
                print(response)
                group = MoveGroupCommander(arm + "_arm")

                # Setting position and orientation target
                group.set_pose_target(request.ik_request.pose_stamped)

                # TRY THIS
                # Setting just the position without specifying the orientation
                #group.set_position_target([0.5, 0.5, 0.0])

                # Plan IK and execute
                group.go()
                
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
            request = GetPositionIKRequest()
            request.ik_request.group_name = arm + "_arm"

            # If a Sawyer does not have a gripper, replace '_gripper_tip' with '_wrist' instead
            link = arm + "_gripper"
            if robo == 'sawyer':
                link += '_tip'

            request.ik_request.ik_link_name = link
            request.ik_request.attempts = 20
            request.ik_request.pose_stamped.header.frame_id = "base"
            
            # Set the desired orientation for the end effector HERE
            request.ik_request.pose_stamped.pose.position.x = x
            request.ik_request.pose_stamped.pose.position.y = y
            request.ik_request.pose_stamped.pose.position.z = -0.14        
            request.ik_request.pose_stamped.pose.orientation.x = 0.0
            request.ik_request.pose_stamped.pose.orientation.y = 1.0
            request.ik_request.pose_stamped.pose.orientation.z = 0.0
            request.ik_request.pose_stamped.pose.orientation.w = 0.0
            
            try:
                # Send the request to the service
                response = compute_ik(request)
                
                # Print the response HERE
                print(response)
                group = MoveGroupCommander(arm + "_arm")

                # Setting position and orientation target
                group.set_pose_target(request.ik_request.pose_stamped)

                # TRY THIS
                # Setting just the position without specifying the orientation
                #group.set_position_target([0.5, 0.5, 0.0])

                # Plan IK and execute
                group.go()
                
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
        request = GetPositionIKRequest()
        request.ik_request.group_name = arm + "_arm"

        # If a Sawyer does not have a gripper, replace '_gripper_tip' with '_wrist' instead
        link = arm + "_gripper"
        if robo == 'sawyer':
            link += '_tip'

        request.ik_request.ik_link_name = link
        request.ik_request.attempts = 20
        request.ik_request.pose_stamped.header.frame_id = "base"
        
        # Set the desired orientation for the end effector HERE
        request.ik_request.pose_stamped.pose.position.x = x
        request.ik_request.pose_stamped.pose.position.y = y
        request.ik_request.pose_stamped.pose.position.z = -0.05        
        request.ik_request.pose_stamped.pose.orientation.x = 0.0
        request.ik_request.pose_stamped.pose.orientation.y = 1.0
        request.ik_request.pose_stamped.pose.orientation.z = 0.0
        request.ik_request.pose_stamped.pose.orientation.w = 0.0
        
        try:
            # Send the request to the service
            response = compute_ik(request)
            
            # Print the response HERE
            print(response)
            group = MoveGroupCommander(arm + "_arm")

            # Setting position and orientation target
            group.set_pose_target(request.ik_request.pose_stamped)

            # TRY THIS
            # Setting just the position without specifying the orientation
            #group.set_position_target([0.5, 0.5, 0.0])

            # Plan IK and execute
            group.go()
            
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

# Python's syntax for a main() method
if __name__ == '__main__':
    main(sys.argv[1])

