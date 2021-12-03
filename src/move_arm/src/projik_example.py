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

def make2(robo, arm, xi, upper, mid, center_x = 0.691, center_y = 0.259):
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
    #draw top of two
    if upper == True:
        # y = -20*((xi-center_x)**2)+center_y
        y = np.sqrt((-xi+center_x)/-20) + center_y
    elif mid == True:
        # y = xi-.741+.209
        # y = xi - center_x - 0.05 + center_y -0.05
        y = -xi + center_y +.05 + center_x + .05
    else:
        # y = center_y -0.15
        y = xi
        xi = center_x + .15
    request.ik_request.pose_stamped.pose.position.x = xi
    request.ik_request.pose_stamped.pose.position.y = y
    request.ik_request.pose_stamped.pose.position.z = -0.147
    request.ik_request.pose_stamped.pose.orientation.x = 0.0
    request.ik_request.pose_stamped.pose.orientation.y = 1.0
    request.ik_request.pose_stamped.pose.orientation.z = 0.0
    request.ik_request.pose_stamped.pose.orientation.w = 0.0
    return request

def make3(robo, arm, xi, upper, xc= 0.691,yc= 0.259):
    request = GetPositionIKRequest()
    request.ik_request.group_name = arm + "_arm"

    # If a Sawyer does not have a gripper, replace '_gripper_tip' with '_wrist' instead
    link = arm + "_gripper"
    if robo == 'sawyer':
        link += '_tip'

    request.ik_request.ik_link_name = link
    request.ik_request.attempts = 20
    request.ik_request.pose_stamped.header.frame_id = "base"
    xi = round(xi,3)
    if upper == True:
        y= round(-30*((xi-xc)**2) +yc,3)
    else:
        y= round(-30*((xi-(xc*1.1))**2) +yc, 3)
    
    request.ik_request.pose_stamped.pose.position.x = xi
    request.ik_request.pose_stamped.pose.position.y = y
    request.ik_request.pose_stamped.pose.position.z = -0.147
    request.ik_request.pose_stamped.pose.orientation.x = 0.0
    request.ik_request.pose_stamped.pose.orientation.y = 1.0
    request.ik_request.pose_stamped.pose.orientation.z = 0.0
    request.ik_request.pose_stamped.pose.orientation.w = 0.0
    return request

def make4(robo, arm, xi, xc= 0.691,yc= 0.259):
    request = GetPositionIKRequest()
    request.ik_request.group_name = arm + "_arm"

    # If a Sawyer does not have a gripper, replace '_gripper_tip' with '_wrist' instead
    link = arm + "_gripper"
    if robo == 'sawyer':
        link += '_tip'

    request.ik_request.ik_link_name = link
    request.ik_request.attempts = 20
    request.ik_request.pose_stamped.header.frame_id = "base"
    if xi == 0:
        request.ik_request.pose_stamped.pose.position.x = xc
        request.ik_request.pose_stamped.pose.position.y = yc
        request.ik_request.pose_stamped.pose.position.z = -0.147
        request.ik_request.pose_stamped.pose.orientation.x = 0.0
        request.ik_request.pose_stamped.pose.orientation.y = 1.0
        request.ik_request.pose_stamped.pose.orientation.z = 0.0
        request.ik_request.pose_stamped.pose.orientation.w = 0.0
    if xi == 1:
        request.ik_request.pose_stamped.pose.position.x = xc+0.0788
        request.ik_request.pose_stamped.pose.position.y = yc
        request.ik_request.pose_stamped.pose.position.z = -0.147
        request.ik_request.pose_stamped.pose.orientation.x = 0.0
        request.ik_request.pose_stamped.pose.orientation.y = 1.0
        request.ik_request.pose_stamped.pose.orientation.z = 0.0
        request.ik_request.pose_stamped.pose.orientation.w = 0.0
    if xi == 2:
        request.ik_request.pose_stamped.pose.position.x = xc+0.0788
        request.ik_request.pose_stamped.pose.position.y = yc + 0.059
        request.ik_request.pose_stamped.pose.position.z = -0.147
        request.ik_request.pose_stamped.pose.orientation.x = 0.0
        request.ik_request.pose_stamped.pose.orientation.y = 1.0
        request.ik_request.pose_stamped.pose.orientation.z = 0.0
        request.ik_request.pose_stamped.pose.orientation.w = 0.0
    if xi == 3:
        request.ik_request.pose_stamped.pose.position.x = xc
        request.ik_request.pose_stamped.pose.position.y = yc + 0.059
        request.ik_request.pose_stamped.pose.position.z = -0.147
        request.ik_request.pose_stamped.pose.orientation.x = 0.0
        request.ik_request.pose_stamped.pose.orientation.y = 1.0
        request.ik_request.pose_stamped.pose.orientation.z = 0.0
        request.ik_request.pose_stamped.pose.orientation.w = 0.0
    if xi == 4:
        request.ik_request.pose_stamped.pose.position.x = xc+0.1577
        request.ik_request.pose_stamped.pose.position.y = yc + 0.059
        request.ik_request.pose_stamped.pose.position.z = -0.147
        request.ik_request.pose_stamped.pose.orientation.x = 0.0
        request.ik_request.pose_stamped.pose.orientation.y = 1.0
        request.ik_request.pose_stamped.pose.orientation.z = 0.0
        request.ik_request.pose_stamped.pose.orientation.w = 0.0
    return request
        

def make5(robo, arm, xi, upper, mid, xc=0.6467, yc=0.2):
    request = GetPositionIKRequest()
    request.ik_request.group_name = arm + "_arm"

    # If a Sawyer does not have a gripper, replace '_gripper_tip' with '_wrist' instead
    link = arm + "_gripper"
    if robo == 'sawyer':
        link += '_tip'

    request.ik_request.ik_link_name = link
    request.ik_request.attempts = 20
    request.ik_request.pose_stamped.header.frame_id = "base"
    xi = round(xi,3)
    if upper == True:
        y = xi 
        xi = xc
    elif mid == True:
        y=yc
    else:
        y= round(-30*((xi-(xc*1.1))**2) +yc, 3)
    
    request.ik_request.pose_stamped.pose.position.x = xi
    request.ik_request.pose_stamped.pose.position.y = y
    request.ik_request.pose_stamped.pose.position.z = -0.147
    request.ik_request.pose_stamped.pose.orientation.x = 0.0
    request.ik_request.pose_stamped.pose.orientation.y = 1.0
    request.ik_request.pose_stamped.pose.orientation.z = 0.0
    request.ik_request.pose_stamped.pose.orientation.w = 0.0
    return request

def make6(robo, arm, xi, upper, mid, xc= 0.73,yc= 1.0):
    request = GetPositionIKRequest()
    request.ik_request.group_name = arm + "_arm"

    # If a Sawyer does not have a gripper, replace '_gripper_tip' with '_wrist' instead
    link = arm + "_gripper"
    if robo == 'sawyer':
        link += '_tip'

    request.ik_request.ik_link_name = link
    request.ik_request.attempts = 20
    request.ik_request.pose_stamped.header.frame_id = "base"
    b = -0.1 
    a = -0.27
    k = yc
    h = xc
    xi = np.round(xi, 3)
    if upper == True:
        #math equatio
        y = np.round((-np.sqrt((b**2)-((b**2/a**2)*((xi-h)**2))))+k, 3)
    else:
        if mid==True:
            y= np.round(np.sqrt((0.1**2)-(xi-xc)**2)+yc, 3)
        else:
            y= np.round(-np.sqrt((0.1**2)-(xi-xc)**2)+yc, 3)
    request.ik_request.pose_stamped.pose.position.x = xi
    request.ik_request.pose_stamped.pose.position.y = y
    request.ik_request.pose_stamped.pose.position.z = -0.147
    request.ik_request.pose_stamped.pose.orientation.x = 0.0
    request.ik_request.pose_stamped.pose.orientation.y = 1.0
    request.ik_request.pose_stamped.pose.orientation.z = 0.0
    request.ik_request.pose_stamped.pose.orientation.w = 0.0
    return request

def make7(robo, arm, xi, upper, xc= 0.6467,yc= 0.2):
    request = GetPositionIKRequest()
    request.ik_request.group_name = arm + "_arm"

    # If a Sawyer does not have a gripper, replace '_gripper_tip' with '_wrist' instead
    link = arm + "_gripper"
    if robo == 'sawyer':
        link += '_tip'

    request.ik_request.ik_link_name = link
    request.ik_request.attempts = 20
    request.ik_request.pose_stamped.header.frame_id = "base"
    xi = round(xi,3)
    if upper:
        y=xi 
        xi = xc 
    else:
        y = yc+0.059
    request.ik_request.pose_stamped.pose.position.x = xi
    request.ik_request.pose_stamped.pose.position.y = y
    request.ik_request.pose_stamped.pose.position.z = -0.147
    request.ik_request.pose_stamped.pose.orientation.x = 0.0
    request.ik_request.pose_stamped.pose.orientation.y = 1.0
    request.ik_request.pose_stamped.pose.orientation.z = 0.0
    request.ik_request.pose_stamped.pose.orientation.w = 0.0
    return request

def make8(robo, arm, xi, upper, mid, center_x = 0.6566, center_y = 0.2235):
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
    #draw top of two
    if upper == True and mid ==True:
        y = -30*((xi-center_x+0.0344)**2)+center_y+0.0355
    elif upper == False and mid == False:
        y = 30*(xi - ((center_x+0.0344)*1.1))**2 + center_y       # y = xi-.741+.209
    elif upper == False and mid == True:
        y = -30*(xi - ((center_x+0.0344)*1.1))**2 + center_y +0.0355
    elif upper == True and mid == False:
        y = 30 * (x-center_x+0.0344)**2 + center_y
    request.ik_request.pose_stamped.pose.position.x = xi
    request.ik_request.pose_stamped.pose.position.y = y
    request.ik_request.pose_stamped.pose.position.z = -0.147
    request.ik_request.pose_stamped.pose.orientation.x = 0.0
    request.ik_request.pose_stamped.pose.orientation.y = 1.0
    request.ik_request.pose_stamped.pose.orientation.z = 0.0
    request.ik_request.pose_stamped.pose.orientation.w = 0.0
    return request
def make9(robo, arm, xi, upper, mid, center_x = 0.6566, center_y = 0.2235):
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
    #draw top of two
    if upper == True and mid ==True:
        y = -30*((xi-center_x)**2)+center_y
    elif upper == False and mid == False:
        y = 30 * (x-center_x)**2 + center_y + 0.071       # y = xi-.741+.209
    elif upper == False and mid == True:
        y = -30 * ((xi-center_x)**2) + center_y
    elif upper == True and mid == False:
        y = center_y + 0.0355
    request.ik_request.pose_stamped.pose.position.x = xi
    request.ik_request.pose_stamped.pose.position.y = y
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
    number = 7 #change this to change the number drawn
    while not rospy.is_shutdown():
        raw_input('Press [ Enter ]: ')
        if number == 9:
            #Computer vision determines start point.
            center_x = 0.6566
            center_y = 0.2235
            # for xi in np.linspace(0.641, 0.741, 3):
            for xi in np.linspace(center_x + 0.0344, center_x + 0.0688, 3):
                request = make9(robo, arm, xi, upper=True, mid=True, center_x = 0.691, center_y = 0.259)
                try:
                    # Send the request to the service
                    response = compute_ik(request)

                    # Print the response HERE
                    # print(response)
                    group = MoveGroupCommander(arm + "_arm")

                    # Setting position and orientation target
                    group.set_pose_target(request.ik_request.pose_stamped)


                    # Plan IK and execute
                    group.go()
                    rospy.sleep(1.0)
            for xi in np.linspace(center_x + 0.0688, center_x, 3):
                request = make9(robo, arm, xi, upper=False, mid=False, center_x, center_y)
                try:
                    # Send the request to the service
                    response = compute_ik(request)

                    # Print the response HERE
                    # print(response)
                    group = MoveGroupCommander(arm + "_arm")

                    # Setting position and orientation target
                    group.set_pose_target(request.ik_request.pose_stamped)


                    # Plan IK and execute
                    group.go()
                    rospy.sleep(1.0)
            for xi in np.linspace(center_x, center_x + 0.0344, 3):
                request = make9(robo, arm, xi, upper=False, mid=True, center_x, center_y)
                try:
                    # Send the request to the service
                    response = compute_ik(request)

                    # Print the response HERE
                    # print(response)
                    group = MoveGroupCommander(arm + "_arm")

                    # Setting position and orientation target
                    group.set_pose_target(request.ik_request.pose_stamped)


                    # Plan IK and execute
                    group.go()
                    rospy.sleep(1.0)

                except rospy.ServiceException, e:
                    print "Service call failed: %s"%e
            for xi in np.linspace(center_x + 0.0344, center_x + 0.1379, 3):
                request = make9(robo, arm, xi, upper=True, mid=False, center_x, center_y)
                try:
                    # Send the request to the service
                    response = compute_ik(request)

                    # Print the response HERE
                    # print(response)
                    group = MoveGroupCommander(arm + "_arm")

                    # Setting position and orientation target
                    group.set_pose_target(request.ik_request.pose_stamped)


                    # Plan IK and execute
                    group.go()
                    rospy.sleep(1.0)

                except rospy.ServiceException, e:
                    print "Service call failed: %s"%e
        if number == 8:
            #Computer vision determines start point.
            center_x = 0.6566
            center_y = 0.2235
            # for xi in np.linspace(0.641, 0.741, 3):
            for xi in np.linspace(center_x, center_x + 0.0688, 3):
                request = make8(robo, arm, xi, upper=True, mid=True, center_x = 0.691, center_y = 0.259)
                try:
                    # Send the request to the service
                    response = compute_ik(request)

                    # Print the response HERE
                    # print(response)
                    group = MoveGroupCommander(arm + "_arm")

                    # Setting position and orientation target
                    group.set_pose_target(request.ik_request.pose_stamped)


                    # Plan IK and execute
                    group.go()
                    rospy.sleep(1.0)
            for xi in np.linspace(center_x + 0.0688, center_x + 0.1379, 3):
                request = make8(robo, arm, xi, upper=False, mid=False, center_x, center_y)
                try:
                    # Send the request to the service
                    response = compute_ik(request)

                    # Print the response HERE
                    # print(response)
                    group = MoveGroupCommander(arm + "_arm")

                    # Setting position and orientation target
                    group.set_pose_target(request.ik_request.pose_stamped)


                    # Plan IK and execute
                    group.go()
                    rospy.sleep(1.0)
            for xi in np.linspace(center_x + 0.1379, center_x + 0.0688, 3):
                request = make8(robo, arm, xi, upper=False, mid=True, center_x, center_y)
                try:
                    # Send the request to the service
                    response = compute_ik(request)

                    # Print the response HERE
                    # print(response)
                    group = MoveGroupCommander(arm + "_arm")

                    # Setting position and orientation target
                    group.set_pose_target(request.ik_request.pose_stamped)


                    # Plan IK and execute
                    group.go()
                    rospy.sleep(1.0)

                except rospy.ServiceException, e:
                    print "Service call failed: %s"%e
            for xi in np.linspace(center_x + 0.0688, center_x, 3):
                request = make8(robo, arm, xi, upper=True, mid=False, center_x, center_y)
                try:
                    # Send the request to the service
                    response = compute_ik(request)

                    # Print the response HERE
                    # print(response)
                    group = MoveGroupCommander(arm + "_arm")

                    # Setting position and orientation target
                    group.set_pose_target(request.ik_request.pose_stamped)


                    # Plan IK and execute
                    group.go()
                    rospy.sleep(1.0)

                except rospy.ServiceException, e:
                    print "Service call failed: %s"%e
        if number == 7:
            center_x = 0.6467
            center_y = 0.2
            for xi in np.linspace(center_y, center_y+0.059, 3):
                request = make7(robo, arm, xi, True, center_x, center_y)
                try:
                    # Send the request to the service
                    response = compute_ik(request)

                    # Print the response HERE
                    # print(response)
                    group = MoveGroupCommander(arm + "_arm")

                    # Setting position and orientation target
                    group.set_pose_target(request.ik_request.pose_stamped)


                    # Plan IK and execute
                    group.go()
                    rospy.sleep(1.0)
                except rospy.ServiceException, e:
                    print "Service call failed: %s"%e
            for xi in np.linspace(center_x, center_x+0.1577, 3):
                request = make7(robo, arm, xi, False, center_x, center_y)
                try:
                    # Send the request to the service
                    response = compute_ik(request)

                    # Print the response HERE
                    # print(response)
                    group = MoveGroupCommander(arm + "_arm")

                    # Setting position and orientation target
                    group.set_pose_target(request.ik_request.pose_stamped)


                    # Plan IK and execute
                    group.go()
                    rospy.sleep(1.0)
                except rospy.ServiceException, e:
                    print "Service call failed: %s"%e
        if number == 6:
            center_x = 0.6467
            center_y = 0.2
            for xi in np.linspace(center_x-0.27, center_x, 3):
                request = make6(robo, arm, xi, True, False, center_x, center_y)
                try:
                    # Send the request to the service
                    response = compute_ik(request)

                    # Print the response HERE
                    # print(response)
                    group = MoveGroupCommander(arm + "_arm")

                    # Setting position and orientation target
                    group.set_pose_target(request.ik_request.pose_stamped)


                    # Plan IK and execute
                    group.go()
                    rospy.sleep(1.0)
                except rospy.ServiceException, e:
                    print "Service call failed: %s"%e
            for xi in np.linspace(center_x, center_x+0.1, 3):
                request = make6(robo, arm, xi, False, False, center_x, center_y)
                try:
                    # Send the request to the service
                    response = compute_ik(request)

                    # Print the response HERE
                    # print(response)
                    group = MoveGroupCommander(arm + "_arm")

                    # Setting position and orientation target
                    group.set_pose_target(request.ik_request.pose_stamped)


                    # Plan IK and execute
                    group.go()
                    rospy.sleep(1.0)
                except rospy.ServiceException, e:
                    print "Service call failed: %s"%e
            for xi in np.linspace(center_x+0.1, center_x-0.1, 3):
                request = make6(robo, arm, xi, False, True, center_x, center_y)
                try:
                    # Send the request to the service
                    response = compute_ik(request)

                    # Print the response HERE
                    # print(response)
                    group = MoveGroupCommander(arm + "_arm")

                    # Setting position and orientation target
                    group.set_pose_target(request.ik_request.pose_stamped)


                    # Plan IK and execute
                    group.go()
                    rospy.sleep(1.0)
                except rospy.ServiceException, e:
                    print "Service call failed: %s"%e
            for xi in np.linspace(center_x-0.1, center_x, 3):
                request = make6(robo, arm, xi, False, False, center_x, center_y)
                try:
                    # Send the request to the service
                    response = compute_ik(request)

                    # Print the response HERE
                    # print(response)
                    group = MoveGroupCommander(arm + "_arm")

                    # Setting position and orientation target
                    group.set_pose_target(request.ik_request.pose_stamped)


                    # Plan IK and execute
                    group.go()
                    rospy.sleep(1.0)
                except rospy.ServiceException, e:
                    print "Service call failed: %s"%e
        if number == 5:
            center_x = 0.6467
            center_y = 0.2
            for xi in np.linspace(center_y+0.059, center_y, 3):
                request = make5(robo, arm, xi, True, False, center_x, center_y)
                try:
                    # Send the request to the service
                    response = compute_ik(request)

                    # Print the response HERE
                    # print(response)
                    group = MoveGroupCommander(arm + "_arm")

                    # Setting position and orientation target
                    group.set_pose_target(request.ik_request.pose_stamped)


                    # Plan IK and execute
                    group.go()
                    rospy.sleep(1.0)
                except rospy.ServiceException, e:
                    print "Service call failed: %s"%e
            for xi in np.linspace(center_x, center_x+0.0691, 3):
                request = make5(robo, arm, xi, False, True, center_x, center_y)
                try:
                    # Send the request to the service
                    response = compute_ik(request)

                    # Print the response HERE
                    # print(response)
                    group = MoveGroupCommander(arm + "_arm")

                    # Setting position and orientation target
                    group.set_pose_target(request.ik_request.pose_stamped)


                    # Plan IK and execute
                    group.go()
                    rospy.sleep(1.0)
                except rospy.ServiceException, e:
                    print "Service call failed: %s"%e
            for xi in np.linspace(center_x+0.0691, center_x+0.1577, 3):
                request = make5(robo, arm, xi, False, False, center_x, center_y)
                try:
                    # Send the request to the service
                    response = compute_ik(request)

                    # Print the response HERE
                    # print(response)
                    group = MoveGroupCommander(arm + "_arm")

                    # Setting position and orientation target
                    group.set_pose_target(request.ik_request.pose_stamped)


                    # Plan IK and execute
                    group.go()
                    rospy.sleep(1.0)
                except rospy.ServiceException, e:
                    print "Service call failed: %s"%e
        if number == 4:
            center_x = 0.6467
            center_y = 0.2
            for xi in range(5):
                request = make4(robo, arm, xi, center_x, center_y)
                try:
                    # Send the request to the service
                    response = compute_ik(request)

                    # Print the response HERE
                    # print(response)
                    group = MoveGroupCommander(arm + "_arm")

                    # Setting position and orientation target
                    group.set_pose_target(request.ik_request.pose_stamped)


                    # Plan IK and execute
                    group.go()
                    rospy.sleep(1.0)
                except rospy.ServiceException, e:
                    print "Service call failed: %s"%e
        if number == 3:
            center_x = 0.691
            center_y = 0.259
            for xi in np.linspace(center_x-0.0443, center_x+0.0345, 3):
                request = make3(robo, arm, xi, True, center_x, center_y)
                try:
                    # Send the request to the service
                    response = compute_ik(request)

                    # Print the response HERE
                    # print(response)
                    group = MoveGroupCommander(arm + "_arm")

                    # Setting position and orientation target
                    group.set_pose_target(request.ik_request.pose_stamped)


                    # Plan IK and execute
                    group.go()
                    rospy.sleep(1.0)
                except rospy.ServiceException, e:
                    print "Service call failed: %s"%e
            for xi in np.linspace(center_x+0.0345, center_x+0.0691, 3):
                request = make3(robo, arm, xi, False, center_x, center_y)
                try:
                    # Send the request to the service
                    response = compute_ik(request)

                    # Print the response HERE
                    # print(response)
                    group = MoveGroupCommander(arm + "_arm")

                    # Setting position and orientation target
                    group.set_pose_target(request.ik_request.pose_stamped)


                    # Plan IK and execute
                    group.go()
                    rospy.sleep(1.0)
                except rospy.ServiceException, e:
                    print "Service call failed: %s"%e
            for xi in np.linspace(center_x+0.0691, center_x+0.1134, 3):
                request = make3(robo, arm, xi, False, center_x, center_y)
                try:
                    # Send the request to the service
                    response = compute_ik(request)

                    # Print the response HERE
                    # print(response)
                    group = MoveGroupCommander(arm + "_arm")

                    # Setting position and orientation target
                    group.set_pose_target(request.ik_request.pose_stamped)


                    # Plan IK and execute
                    group.go()
                    rospy.sleep(1.0)
                except rospy.ServiceException, e:
                    print "Service call failed: %s"%e
        if number == 2:
            #Computer vision determines start point.
            center_x = 0.691
            center_y = 0.259
            # for xi in np.linspace(0.641, 0.741, 3):
            for xi in np.linspace(center_x, center_x + 0.05, 3):
                request = make2(robo, arm, xi, True, False, center_x, center_y)
                try:
                    # Send the request to the service
                    response = compute_ik(request)

                    # Print the response HERE
                    # print(response)
                    group = MoveGroupCommander(arm + "_arm")

                    # Setting position and orientation target
                    group.set_pose_target(request.ik_request.pose_stamped)


                    # Plan IK and execute
                    group.go()
                    rospy.sleep(1.0)
                except rospy.ServiceException, e:
                    print "Service call failed: %s"%e
            for xi in np.linspace(center_x + 0.05, center_x + 0.15, 3):
                request = make2(robo, arm, xi, False, True, center_x, center_y)
                try:
                    # Send the request to the service
                    response = compute_ik(request)

                    # Print the response HERE
                    # print(response)
                    group = MoveGroupCommander(arm + "_arm")

                    # Setting position and orientation target
                    group.set_pose_target(request.ik_request.pose_stamped)


                    # Plan IK and execute
                    group.go()
                    rospy.sleep(1.0)
                except rospy.ServiceException, e:
                    print "Service call failed: %s"%e
            for xi in np.linspace(center_y - 0.05, center_y + 0.05, 3):
                request = make2(robo, arm, xi, False, False, center_x, center_y)
                try:
                    # Send the request to the service
                    response = compute_ik(request)

                    # Print the response HERE
                    # print(response)
                    group = MoveGroupCommander(arm + "_arm")

                    # Setting position and orientation target
                    group.set_pose_target(request.ik_request.pose_stamped)


                    # Plan IK and execute
                    group.go()
                    rospy.sleep(1.0)

                except rospy.ServiceException, e:
                    print "Service call failed: %s"%e
        # Set the desired orientation for the end effector HERE
        if number==1:
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
        if number == 0:
            for xi in np.linspace(0.692, 0.765, 3):
                request = make0(robo, arm, xi, True)
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
            for xi in np.linspace(0.765, 0.838, 3):
                request = make0(robo, arm, xi, True)
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

            for xi in np.linspace(0.838, 0.765, 3):
                print("new")
                request = make0(robo, arm, xi, False)
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
            for xi in np.linspace(0.765, 0.692, 3):
                request = make0(robo, arm, xi, False)
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
# Python's syntax for a main() method
if __name__ == '__main__':
    if sys.argv[1] == 'sawyer':
        from intera_interface import gripper as robot_gripper
    else:
        from baxter_interface import gripper as robot_gripper

    main(sys.argv[1])

