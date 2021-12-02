#!/usr/bin/env python
import rospy
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander
import numpy as np
from numpy import linalg
import sys

def make0(robo, arm, xi, upper):
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

    # print('Opening...')
    # right_gripper.open()
    # rospy.sleep(1.0)
    # print('Done!')
    b = 0.048 
    a = 0.068
    k = 0.193
    h = 0.765
    xi = np.round(xi, 3)
    if upper == True:
        #math equatio
        y = np.round((np.sqrt((b**2)-((b**2/a**2)*((xi-h)**2))))+k, 3)
        print(xi)
        print(y)
        if np.isnan(y):
            y = 0.193
            if xi == 0.765:
                y = 0.293
        #y = np.round(((b/a)*np.sqrt(((a**2)-((xi-h)**2)))+k), 3)
    else:
        print("im here")
        y = np.round((-1*np.sqrt((b**2)-((b**2/a**2)*((xi-h)**2))))+k, 3)
        print(xi)
        print(y)
        if np.isnan(y):
            y = 0.193
            if xi == 0.765:
                y = 0.093
        #y = np.round(((b/a)*-np.sqrt(((a**2)-((xi-h)**2)))+k), 3)
    # Set the desired orientation for the end effector HERE
    #switch = not switch
    request.ik_request.pose_stamped.pose.position.x = xi
    request.ik_request.pose_stamped.pose.position.y = y
    request.ik_request.pose_stamped.pose.position.z = -0.147        
    request.ik_request.pose_stamped.pose.orientation.x = 0.0
    request.ik_request.pose_stamped.pose.orientation.y = 1.0
    request.ik_request.pose_stamped.pose.orientation.z = 0.0
    request.ik_request.pose_stamped.pose.orientation.w = 0.0
    return request

def make1(robo, arm, xi, upper):
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
    if upper == False:
        request.ik_request.pose_stamped.pose.position.x = 0.692
        request.ik_request.pose_stamped.pose.position.y = 0.093
        request.ik_request.pose_stamped.pose.position.z = -0.147        
        request.ik_request.pose_stamped.pose.orientation.x = 0.0
        request.ik_request.pose_stamped.pose.orientation.y = 1.0
        request.ik_request.pose_stamped.pose.orientation.z = 0.0
        request.ik_request.pose_stamped.pose.orientation.w = 0.0
    else:
        request.ik_request.pose_stamped.pose.position.x = 0.838
        request.ik_request.pose_stamped.pose.position.y = 0.093
        request.ik_request.pose_stamped.pose.position.z = -0.147        
        request.ik_request.pose_stamped.pose.orientation.x = 0.0
        request.ik_request.pose_stamped.pose.orientation.y = 1.0
        request.ik_request.pose_stamped.pose.orientation.z = 0.0
        request.ik_request.pose_stamped.pose.orientation.w = 0.0
    return request

        




def main(robo):
    # Wait for the IK service to become available
    rospy.wait_for_service('compute_ik')
    rospy.init_node('service_query')

# Set up the right gripper
    right_gripper = robot_gripper.Gripper('right')

    # Calibrate the gripper (other commands won't work unless you do this first)
    print('Calibrating...')
    right_gripper.calibrate()
    rospy.sleep(2.0)

    arm = 'left'
    # Create the function used to call the service
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
    if robo == 'sawyer':
    	arm = 'right'
    switch = True
    while not rospy.is_shutdown():
        raw_input('Press [ Enter ]: ')
        # Set the desired orientation for the end effector HERE
        request = make1(robo, arm, 0.692, False)
        try:
            # Send the request to the service
            response = compute_ik(request)
            

            # Print the response HERE
            # print(response)
            group = MoveGroupCommander(arm + "_arm")

            # Setting position and orientation target
            group.set_pose_target(request.ik_request.pose_stamped)

            # TRY THIS
            # Setting just the position without specifying the orientation
            # group.set_position_target([0.5, 0.5, 0.0])

            # Plan IK and execute
            group.go()
            rospy.sleep(1.0)

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        request = make1(robo, arm, 0.692, True)
        try:
            # Send the request to the service
            response = compute_ik(request)
            

            # Print the response HERE
            # print(response)
            group = MoveGroupCommander(arm + "_arm")

            # Setting position and orientation target
            group.set_pose_target(request.ik_request.pose_stamped)

            # TRY THIS
            # Setting just the position without specifying the orientation
            # group.set_position_target([0.5, 0.5, 0.0])

            # Plan IK and execute
            group.go()
            rospy.sleep(1.0)

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        #if switch == True:
            # for xi in np.linspace(0.692, 0.765, 3):
            #     request = make0(robo, arm, xi, True)
            #     try:
            #         # Send the request to the service
            #         response = compute_ik(request)
                    

            #         # Print the response HERE
            #         # print(response)
            #         group = MoveGroupCommander(arm + "_arm")

            #         # Setting position and orientation target
            #         group.set_pose_target(request.ik_request.pose_stamped)

            #         # TRY THIS
            #         # Setting just the position without specifying the orientation
            #         # group.set_position_target([0.5, 0.5, 0.0])

            #         # Plan IK and execute
            #         group.go()
            #         rospy.sleep(1.0)

            #     except rospy.ServiceException, e:
            #         print "Service call failed: %s"%e
            # for xi in np.linspace(0.765, 0.838, 3):
            #     request = make0(robo, arm, xi, True)
            #     try:
            #         # Send the request to the service
            #         response = compute_ik(request)
                    

            #         # Print the response HERE
            #         # print(response)
            #         group = MoveGroupCommander(arm + "_arm")

            #         # Setting position and orientation target
            #         group.set_pose_target(request.ik_request.pose_stamped)

            #         # TRY THIS
            #         # Setting just the position without specifying the orientation
            #         # group.set_position_target([0.5, 0.5, 0.0])

            #         # Plan IK and execute
            #         group.go()
            #         rospy.sleep(1.0)

            #     except rospy.ServiceException, e:
            #         print "Service call failed: %s"%e

            # for xi in np.linspace(0.838, 0.765, 3):
            #     print("new")
            #     request = make0(robo, arm, xi, False)
            #     try:
            #         # Send the request to the service
            #         response = compute_ik(request)
                    

            #         # Print the response HERE
            #         # print(response)
            #         group = MoveGroupCommander(arm + "_arm")

            #         # Setting position and orientation target
            #         group.set_pose_target(request.ik_request.pose_stamped)

            #         # TRY THIS
            #         # Setting just the position without specifying the orientation
            #         # group.set_position_target([0.5, 0.5, 0.0])

            #         # Plan IK and execute
            #         group.go()
            #         rospy.sleep(1.0)

            #     except rospy.ServiceException, e:
            #         print "Service call failed: %s"%e
            # for xi in np.linspace(0.765, 0.692, 3):
            #     request = make0(robo, arm, xi, False)
            #     try:
            #         # Send the request to the service
            #         response = compute_ik(request)
                    

            #         # Print the response HERE
            #         # print(response)
            #         group = MoveGroupCommander(arm + "_arm")

            #         # Setting position and orientation target
            #         group.set_pose_target(request.ik_request.pose_stamped)

            #         # TRY THIS
            #         # Setting just the position without specifying the orientation
            #         # group.set_position_target([0.5, 0.5, 0.0])

            #         # Plan IK and execute
            #         group.go()
            #         rospy.sleep(1.0)

            #     except rospy.ServiceException, e:
            #         print "Service call failed: %s"%e
# Python's syntax for a main() method
if __name__ == '__main__':
    if sys.argv[1] == 'sawyer':
        from intera_interface import gripper as robot_gripper
    else:
        from baxter_interface import gripper as robot_gripper

    main(sys.argv[1])

