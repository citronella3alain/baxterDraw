#!/usr/bin/env python

#Create a virtual environment to use this file:
# source env/bin/activate
import rospy
from sensor_msgs.msg import Image
from lab4_cam.srv import ImageSrv, ImageSrvResponse
import cv2, time, sys
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import re
from numpy.linalg import *
import requests
import json
import re
import tf
from geometry_msgs.msg import PoseStamped

from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander
from numpy import linalg
import sys
import moveit_commander
from copy import deepcopy

def make0(robo, arm, xi, upper, center_x, center_y):
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
    # 
    # print('Done!')
    b = 0.048
    a = 0.068
    k = center_y #0.193
    h = center_x+0.073
    xi = np.round(xi, 3)
    if upper == True:
        #math equatio
        y = np.round((np.sqrt((b**2)-((b**2/a**2)*((xi-h)**2))))+k, 3)
        print(xi)
        print(y)
        if np.isnan(y):
            y = center_y
            if xi == center_x + 0.073:
                y = center_y + 0.1
        #y = np.round(((b/a)*np.sqrt(((a**2)-((xi-h)**2)))+k), 3)
    else:
        print("im here")
        y = np.round((-1*np.sqrt((b**2)-((b**2/a**2)*((xi-h)**2))))+k, 3)
        print(xi)
        print(y)
        if np.isnan(y):
            y = center_y
            if xi == center_x + 0.073:
                y = center_y -0.1
        #y = np.round(((b/a)*-np.sqrt(((a**2)-((xi-h)**2)))+k), 3)
    # Set the desired orientation for the end effector HERE
    #switch = not switch
    request.ik_request.pose_stamped.pose.position.x = xi
    request.ik_request.pose_stamped.pose.position.y = y
    request.ik_request.pose_stamped.pose.position.z = -0.13
    request.ik_request.pose_stamped.pose.orientation.x = 0.0
    request.ik_request.pose_stamped.pose.orientation.y = 1.0
    request.ik_request.pose_stamped.pose.orientation.z = 0.0
    request.ik_request.pose_stamped.pose.orientation.w = 0.0
    return request.ik_request.pose_stamped.pose

def make1(robo, arm, xi, xc, yc):
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
    y = yc+0.059
    request.ik_request.pose_stamped.pose.position.x = xi
    request.ik_request.pose_stamped.pose.position.y = y
    request.ik_request.pose_stamped.pose.position.z = -0.13
    request.ik_request.pose_stamped.pose.orientation.x = 0.0
    request.ik_request.pose_stamped.pose.orientation.y = 1.0
    request.ik_request.pose_stamped.pose.orientation.z = 0.0
    request.ik_request.pose_stamped.pose.orientation.w = 0.0
    return request.ik_request.pose_stamped.pose

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
    if upper == True and mid == True:
        y = -np.sqrt((-xi+center_x)/-20) + center_y
    elif upper == True:
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
    request.ik_request.pose_stamped.pose.position.z = -0.13
    request.ik_request.pose_stamped.pose.orientation.x = 0.0
    request.ik_request.pose_stamped.pose.orientation.y = 1.0
    request.ik_request.pose_stamped.pose.orientation.z = 0.0
    request.ik_request.pose_stamped.pose.orientation.w = 0.0
    return request.ik_request.pose_stamped.pose

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
    request.ik_request.pose_stamped.pose.position.z = -0.13
    request.ik_request.pose_stamped.pose.orientation.x = 0.0
    request.ik_request.pose_stamped.pose.orientation.y = 1.0
    request.ik_request.pose_stamped.pose.orientation.z = 0.0
    request.ik_request.pose_stamped.pose.orientation.w = 0.0
    return request.ik_request.pose_stamped.pose

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
        request.ik_request.pose_stamped.pose.position.z = -0.13
        request.ik_request.pose_stamped.pose.orientation.x = 0.0
        request.ik_request.pose_stamped.pose.orientation.y = 1.0
        request.ik_request.pose_stamped.pose.orientation.z = 0.0
        request.ik_request.pose_stamped.pose.orientation.w = 0.0
    if xi == 1:
        request.ik_request.pose_stamped.pose.position.x = xc+(0.0788/2.0)
        request.ik_request.pose_stamped.pose.position.y = yc
        request.ik_request.pose_stamped.pose.position.z = -0.13
        request.ik_request.pose_stamped.pose.orientation.x = 0.0
        request.ik_request.pose_stamped.pose.orientation.y = 1.0
        request.ik_request.pose_stamped.pose.orientation.z = 0.0
        request.ik_request.pose_stamped.pose.orientation.w = 0.0
    if xi == 2:
        request.ik_request.pose_stamped.pose.position.x = xc+0.0788
        request.ik_request.pose_stamped.pose.position.y = yc
        request.ik_request.pose_stamped.pose.position.z = -0.13
        request.ik_request.pose_stamped.pose.orientation.x = 0.0
        request.ik_request.pose_stamped.pose.orientation.y = 1.0
        request.ik_request.pose_stamped.pose.orientation.z = 0.0
        request.ik_request.pose_stamped.pose.orientation.w = 0.0
    if xi == 3:
        request.ik_request.pose_stamped.pose.position.x = xc+0.0788
        request.ik_request.pose_stamped.pose.position.y = yc + 0.059
        request.ik_request.pose_stamped.pose.position.z = -0.13
        request.ik_request.pose_stamped.pose.orientation.x = 0.0
        request.ik_request.pose_stamped.pose.orientation.y = 1.0
        request.ik_request.pose_stamped.pose.orientation.z = 0.0
        request.ik_request.pose_stamped.pose.orientation.w = 0.0
    if xi == 4:
        request.ik_request.pose_stamped.pose.position.x = xc
        request.ik_request.pose_stamped.pose.position.y = yc + 0.059
        request.ik_request.pose_stamped.pose.position.z = -0.13
        request.ik_request.pose_stamped.pose.orientation.x = 0.0
        request.ik_request.pose_stamped.pose.orientation.y = 1.0
        request.ik_request.pose_stamped.pose.orientation.z = 0.0
        request.ik_request.pose_stamped.pose.orientation.w = 0.0
    if xi == 5:
        request.ik_request.pose_stamped.pose.position.x = xc+(0.1577/2.0)
        request.ik_request.pose_stamped.pose.position.y = yc + 0.059
        request.ik_request.pose_stamped.pose.position.z = -0.13
        request.ik_request.pose_stamped.pose.orientation.x = 0.0
        request.ik_request.pose_stamped.pose.orientation.y = 1.0
        request.ik_request.pose_stamped.pose.orientation.z = 0.0
        request.ik_request.pose_stamped.pose.orientation.w = 0.0
    if xi == 6:
        request.ik_request.pose_stamped.pose.position.x = xc+0.1577
        request.ik_request.pose_stamped.pose.position.y = yc + 0.059
        request.ik_request.pose_stamped.pose.position.z = -0.13
        request.ik_request.pose_stamped.pose.orientation.x = 0.0
        request.ik_request.pose_stamped.pose.orientation.y = 1.0
        request.ik_request.pose_stamped.pose.orientation.z = 0.0
        request.ik_request.pose_stamped.pose.orientation.w = 0.0
    return request.ik_request.pose_stamped.pose

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
        y= round(-30*((xi-((xc+0.0344)*1.1))**2) + yc + 0.071, 3)

    request.ik_request.pose_stamped.pose.position.x = xi
    request.ik_request.pose_stamped.pose.position.y = y
    request.ik_request.pose_stamped.pose.position.z = -0.13
    request.ik_request.pose_stamped.pose.orientation.x = 0.0
    request.ik_request.pose_stamped.pose.orientation.y = 1.0
    request.ik_request.pose_stamped.pose.orientation.z = 0.0
    request.ik_request.pose_stamped.pose.orientation.w = 0.0
    return request.ik_request.pose_stamped.pose

def make6(robo, arm, xi, upper, center_x = 0.6566, center_y = 0.2235):
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
    if upper == 0:
        y = 30 * (xi-(center_x+0.0344))**2 + center_y - 0.0355
    elif upper == 1:
        y=center_y-0.0355
    elif upper == 2:
        y = 30*(xi - ((center_x+0.0344)*1.1))**2 + center_y - 0.0355
    elif upper == 3:
        y = -30*(xi - ((center_x+0.0344)*1.1))**2 + center_y +0.0355
    elif upper==4:
        y = 30*(xi - ((center_x+0.0344)*1.1))**2 + center_y - 0.0355
    request.ik_request.pose_stamped.pose.position.x = xi
    request.ik_request.pose_stamped.pose.position.y = y
    request.ik_request.pose_stamped.pose.position.z = -0.13
    request.ik_request.pose_stamped.pose.orientation.x = 0.0
    request.ik_request.pose_stamped.pose.orientation.y = 1.0
    request.ik_request.pose_stamped.pose.orientation.z = 0.0
    request.ik_request.pose_stamped.pose.orientation.w = 0.0
    return request.ik_request.pose_stamped.pose

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
    request.ik_request.pose_stamped.pose.position.z = -0.13
    request.ik_request.pose_stamped.pose.orientation.x = 0.0
    request.ik_request.pose_stamped.pose.orientation.y = 1.0
    request.ik_request.pose_stamped.pose.orientation.z = 0.0
    request.ik_request.pose_stamped.pose.orientation.w = 0.0
    return request.ik_request.pose_stamped.pose

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
        y = -30*((xi-(center_x+0.0344))**2)+center_y+0.0355
    elif upper == False and mid == False:
        y = 30*(xi - ((center_x+0.0344)*1.1))**2 + center_y - 0.0355       # y = xi-.741+.209
    elif upper == False and mid == True:
        y = -30*(xi - ((center_x+0.0344)*1.1))**2 + center_y +0.0355
    elif upper == True and mid == False:
        y = 30 * (xi-(center_x+0.0344))**2 + center_y - 0.0355
    request.ik_request.pose_stamped.pose.position.x = xi
    request.ik_request.pose_stamped.pose.position.y = y
    request.ik_request.pose_stamped.pose.position.z = -0.13
    request.ik_request.pose_stamped.pose.orientation.x = 0.0
    request.ik_request.pose_stamped.pose.orientation.y = 1.0
    request.ik_request.pose_stamped.pose.orientation.z = 0.0
    request.ik_request.pose_stamped.pose.orientation.w = 0.0
    return request.ik_request.pose_stamped.pose
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
        y = -30*((xi-(center_x+0.0344))**2)+center_y+0.0355
    elif upper == False and mid == False:
        y = 30 * (xi-(center_x+0.0344))**2 + center_y - 0.0355       # y = xi-.741+.209
    elif upper == False and mid == True:
        y = -30*((xi-(center_x+0.0344))**2)+center_y+0.0355
    elif upper == True and mid == False:
        y = center_y + 0.0355
    request.ik_request.pose_stamped.pose.position.x = xi
    request.ik_request.pose_stamped.pose.position.y = y
    request.ik_request.pose_stamped.pose.position.z = -0.13
    request.ik_request.pose_stamped.pose.orientation.x = 0.0
    request.ik_request.pose_stamped.pose.orientation.y = 1.0
    request.ik_request.pose_stamped.pose.orientation.z = 0.0
    request.ik_request.pose_stamped.pose.orientation.w = 0.0
    return request.ik_request.pose_stamped.pose


# Create a CvBridge to convert ROS messages to OpenCV images
bridge = CvBridge()

# Converts a ROS Image message to a NumPy array to be displayed by OpenCV
def ros_to_np_img(ros_img_msg):
    return np.array(bridge.imgmsg_to_cv2(ros_img_msg,'bgr8'))


if __name__ == '__main__':
    if sys.argv[1] == 'sawyer':
        from intera_interface import gripper as robot_gripper
    else:
        from baxter_interface import gripper as robot_gripper


# Waits for the image service to become available
rospy.wait_for_service('last_image')

# Creates a function used to call the
# image capture service: ImageSrv is the service type
last_image_service = rospy.ServiceProxy('last_image', ImageSrv)

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
robo = 'sawyer'
if robo == 'sawyer':
    arm = 'right'
switch = True
number = 0 #change this to change the number drawn
while not rospy.is_shutdown():
    try:
        # Waits for a key input to continue
        raw_input('Press enter to capture an image:')
    except KeyboardInterrupt:
        print('Break from raw_input')
        break

    try:
        # Request the last image from the image service
        # And extract the ROS Image from the ImageSrv service
        # Remember that ImageSrv.image_data was
        # defined to be of type sensor_msgs.msg.Image
        ros_img_msg = last_image_service().image_data

        # Convert the ROS message to a NumPy image
        np_image = ros_to_np_img(ros_img_msg)

        # Display the CV Image
        cv2.imshow("CV Image", np_image)
        api_key = json.load(open('/home/cc/ee106a/fl21/class/ee106a-aeg/ros_workspaces/baxterDraw/src/lab4_cam/src/app_key.json',))
        r = requests.post("https://api.mathpix.com/v3/latex",
        files={"file": cv2.imencode('.jpg', np_image)[1].tobytes()},
        data={"options_json": json.dumps({
            "formats": ["latex_simplified", "asciimath"]})}, headers=api_key)

        print(json.dumps(r.json(), indent=4, sort_keys=True))
        print("__________________________________")

        hopefully_dict = json.loads(json.dumps(r.json(), indent=4, sort_keys=True))

        print(hopefully_dict)

        latex_vals = hopefully_dict['latex_simplified']

        print(latex_vals)

        number = 0 #change this to change the number drawn for debugging without camera
        ## Parses the string and evaluates
        string = latex_vals.replace(" ", "")
        if string[0] == 'y':
            ##Must use y= at front for equations. evaluated in the if statement below
            number = 'eq'
        elif string[-1]== '=':
            ##Gets rid of equal sign at the end. Must have equal sign or will delete info!!
            number = eval(string[0:-1])
        elif "x" in string:
            number = 'eq'
            print('bad format, use y= at beginning')
        else:
            number = eval(string)
            print('bad format, use = at end')
        raw_input('Press [ Enter ]: ')
        position = hopefully_dict["position"]
        center_x = 0.708
        center_y = -0.091
        x_cam = float(position["top_left_x"]) + float(position["width"])
        y_cam = float(position["top_left_y"])
        print("xcam " + str(x_cam))
        print("ycam " + str(y_cam))
        center_x = (9*(10**-6)*(y_cam**2))-(0.0011*y_cam)+0.698 #change this for each camera calibration given by excel sheet
        center_y = (10**-7)*(x_cam**2)+(0.001*x_cam)-0.7944 #change this for each camera calibration given by excel sheet
        center_x -= 0.02 #offset from equal sign moves the number above it.
        print('center_x: ' + str(center_x))
        print('center_y: ' + str(center_y))
        if number == 'eq':
            group = MoveGroupCommander(arm + "_arm")
            group.allow_replanning(True)
            waypoints = []
            #need board size in coordinates, origin for calculating offset, window size we want to show, aspect ratio
            # x = -y and y = x in the robot's frame to draw the function facing us
            #TODO:
            boardSizeVert = 0.348
            window_size = 10
            minx = 0.513
            maxx = 0.861
            miny = -0.16
            maxy = 0.457
            graphcenx = 0
            graphceny = 0
            originx_board = 0.687
            originy_board =  0.1485
            resolution = 80
            coords = []
            scaling_fact = window_size/boardSizeVert
            print("LOWER BOUND")
            print(graphcenx - window_size)
            print("UPPER BOUND")
            print(graphcenx + window_size)
            for xi in np.linspace(graphcenx - window_size, graphcenx + window_size, resolution):
                y = None
                if string[0] == 'y':
                    print("y = ")
                    print(re.sub("x", '('+ str(xi) +')', string)[2:])
                    y = eval(re.sub("x", '('+ str(xi)+')', string)[2:])
                else:
                    y = eval(re.sub("x", '('+ str(xi)+')', string))
                if np.abs(y) <= window_size or not np.isnan(y):
                    print(xi)
                    print(y)
                    xval = -(y/scaling_fact) + originx_board
                    yval = (xi/scaling_fact) + originy_board
                    print(xval)
                    print(yval)
                    if xval < minx:
                        xval = minx
                    elif xval> maxx:
                        xval = maxx
                    if yval <miny:
                        yval= miny
                    elif yval >maxy:
                        yval = maxy
                    coords.append((xval, yval))
            # Find max of all values in tuple, then do max_val/boardSizeVert to get scaling_fact. divide all numbers by this scaling_fact
            request = GetPositionIKRequest()
            request.ik_request.group_name = arm + "_arm"
            # If a Sawyer does not have a gripper, replace '_gripper_tip' with '_wrist' instead
            link = arm + "_gripper"
            if robo == 'sawyer':
                link += '_tip'
            request.ik_request.ik_link_name = link
            request.ik_request.attempts = 20
            request.ik_request.pose_stamped.header.frame_id = "base"
            request.ik_request.pose_stamped.pose.position.x = originx_board
            request.ik_request.pose_stamped.pose.position.y = originy_board
            request.ik_request.pose_stamped.pose.position.z = -0.05
            request.ik_request.pose_stamped.pose.orientation.x = 0.0
            request.ik_request.pose_stamped.pose.orientation.y = 1.0
            request.ik_request.pose_stamped.pose.orientation.z = 0.0
            request.ik_request.pose_stamped.pose.orientation.w = 0.0
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
                
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
            request = GetPositionIKRequest()
            request.ik_request.group_name = arm + "_arm"
            # If a Sawyer does not have a gripper, replace '_gripper_tip' with '_wrist' instead
            link = arm + "_gripper"
            if robo == 'sawyer':
                link += '_tip'
            z = -0.13
            request.ik_request.ik_link_name = link
            request.ik_request.attempts = 20
            request.ik_request.pose_stamped.header.frame_id = "base"
            request.ik_request.pose_stamped.pose.position.x = originx_board
            request.ik_request.pose_stamped.pose.position.y = originy_board
            request.ik_request.pose_stamped.pose.position.z = z
            request.ik_request.pose_stamped.pose.orientation.x = 0.0
            request.ik_request.pose_stamped.pose.orientation.y = 1.0
            request.ik_request.pose_stamped.pose.orientation.z = 0.0
            request.ik_request.pose_stamped.pose.orientation.w = 0.0
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
            request.ik_request.pose_stamped.pose.position.x = originx_board
            request.ik_request.pose_stamped.pose.position.y = originy_board
            request.ik_request.pose_stamped.pose.position.z = -0.05
            request.ik_request.pose_stamped.pose.orientation.x = 0.0
            request.ik_request.pose_stamped.pose.orientation.y = 1.0
            request.ik_request.pose_stamped.pose.orientation.z = 0.0
            request.ik_request.pose_stamped.pose.orientation.w = 0.0
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
                
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
            group = MoveGroupCommander(arm + "_arm")
            group.allow_replanning(True)
            waypoints =[]
            for x, y in coords:
                request = GetPositionIKRequest()
                request.ik_request.group_name = arm + "_arm"
                # If a Sawyer does not have a gripper, replace '_gripper_tip' with '_wrist' instead
                link = arm + "_gripper"
                if robo == 'sawyer':
                    link += '_tip'
                z = -0.13
                if x == minx or x == maxx or y== miny or y ==maxy:
                    z=-0.05
                request.ik_request.ik_link_name = link
                request.ik_request.attempts = 20
                request.ik_request.pose_stamped.header.frame_id = "base"
                request.ik_request.pose_stamped.pose.position.x = round(x, 3)
                request.ik_request.pose_stamped.pose.position.y = round(y, 3)
                request.ik_request.pose_stamped.pose.position.z = z
                request.ik_request.pose_stamped.pose.orientation.x = 0.0
                request.ik_request.pose_stamped.pose.orientation.y = 1.0
                request.ik_request.pose_stamped.pose.orientation.z = 0.0
                request.ik_request.pose_stamped.pose.orientation.w = 0.0
                waypoints.append(request.ik_request.pose_stamped.pose)
            try:
                robot = moveit_commander
                (plan, fraction) = group.compute_cartesian_path(waypoints, 0.0001, 0.0, True)
                print(fraction)
                robot = moveit_commander.RobotCommander()
                plan = group.retime_trajectory(robot.get_current_state(), plan, 1.0)
                group.execute(plan, wait=True)
                rospy.sleep(0.1)
                group.stop()
                
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
        else:
            for digit in str(number):
                ### Lift hand
                ##spacing between numbers
                digit = int(digit)
                print(digit)
                print('this is the digit')
                center_y += 0.1
                center_x = center_x
                z = -0.05
                request = GetPositionIKRequest()
                request.ik_request.group_name = arm + "_arm"
                # If a Sawyer does not have a gripper, replace '_gripper_tip' with '_wrist' instead
                link = arm + "_gripper"
                if robo == 'sawyer':
                    link += '_tip'
                request.ik_request.ik_link_name = link
                request.ik_request.attempts = 20
                request.ik_request.pose_stamped.header.frame_id = "base"
                request.ik_request.pose_stamped.pose.position.x = round(center_x, 3)
                request.ik_request.pose_stamped.pose.position.y = round(center_y, 3)
                request.ik_request.pose_stamped.pose.position.z = z
                request.ik_request.pose_stamped.pose.orientation.x = 0.0
                request.ik_request.pose_stamped.pose.orientation.y = 1.0
                request.ik_request.pose_stamped.pose.orientation.z = 0.0
                request.ik_request.pose_stamped.pose.orientation.w = 0.0
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
                    
                except rospy.ServiceException, e:
                    print "Service call failed: %s"%e

                if digit == 9:
                    group = MoveGroupCommander(arm + "_arm")
                    group.allow_replanning(True)
                    waypoints =[]
                    waypoints.append(make9(robo, arm, center_x + 0.0344, True, False, center_x, center_y))
                        print "Service call failed: %s"%e
                    for xi in np.linspace(center_x + 0.0688, center_x, 3):
                        waypoints.append(make9(robo, arm, xi, False, False, center_x, center_y))
                    for xi in np.linspace(center_x, center_x + 0.0344, 3):
                        waypoints.append(make9(robo, arm, xi, False, True, center_x, center_y))
                    for xi in np.linspace(center_x + 0.0344, center_x + 0.1379, 3):
                        waypoints.append(make9(robo, arm, xi, True, False, center_x, center_y))
                    try:
                        robot = moveit_commander
                        (plan, fraction) = group.compute_cartesian_path(waypoints, 0.0001, 0.0, True)
                        print(fraction)
                        robot = moveit_commander.RobotCommander()
                        plan = group.retime_trajectory(robot.get_current_state(), plan, 1.0)
                        group.execute(plan, wait=True)
                        rospy.sleep(0.1)
                        group.stop()
                    except rospy.ServiceException, e:
                        print "Service call failed: %s"%e
                elif digit == 8:
                    group = MoveGroupCommander(arm + "_arm")
                    group.allow_replanning(True)
                    waypoints =[]
                    for xi in np.linspace(center_x, center_x + 0.0688, 3):
                        waypoints.append(make8(robo, arm, xi, True, True, center_x, center_y))
                    for xi in np.linspace(center_x + 0.0688, center_x + 0.1379, 3):
                        waypoints.append(make8(robo, arm, xi, False, False, center_x, center_y))
                    for xi in np.linspace(center_x + 0.1379, center_x + 0.0688, 3):
                        waypoints.append(make8(robo, arm, xi, False, True, center_x, center_y))
                    for xi in np.linspace(center_x + 0.0688, center_x, 3):
                        waypoints.append(make8(robo, arm, xi, True, False, center_x, center_y))
                    try:
                        robot = moveit_commander
                        (plan, fraction) = group.compute_cartesian_path(waypoints, 0.0001, 0.0, True)
                        print(fraction)
                        robot = moveit_commander.RobotCommander()
                        plan = group.retime_trajectory(robot.get_current_state(), plan, 1.0)
                        group.execute(plan, wait=True)
                        rospy.sleep(0.1)
                        group.stop()
                    except rospy.ServiceException, e:
                        print "Service call failed: %s"%e
                elif digit == 7:
                    group = MoveGroupCommander(arm + "_arm")
                    group.allow_replanning(True)
                    waypoints =[]
                    for xi in np.linspace(center_y, center_y+0.059, 3):
                        waypoints.append(make7(robo, arm, xi, True, center_x, center_y))
                    for xi in np.linspace(center_x, center_x+0.1577, 3):
                        waypoints.append(make7(robo, arm, xi, False, center_x, center_y))
                    try:
                        robot = moveit_commander
                        (plan, fraction) = group.compute_cartesian_path(waypoints, 0.0001, 0.0, True)
                        print(fraction)
                        robot = moveit_commander.RobotCommander()
                        plan = group.retime_trajectory(robot.get_current_state(), plan, 1.0)
                        group.execute(plan, wait=True)
                        rospy.sleep(0.1)
                        group.stop()
                    except rospy.ServiceException, e:
                        print "Service call failed: %s"%e
                elif digit == 6:
                    group = MoveGroupCommander(arm + "_arm")
                    group.allow_replanning(True)
                    waypoints =[]
                    for xi in np.linspace(center_x, center_x + 0.0344, 3):
                        waypoints.append(make6(robo, arm, xi, 0, center_x, center_y))
                    for xi in np.linspace(center_x + 0.0344, center_x + 0.1035, 3):
                        waypoints.append(make6(robo, arm, xi, 1, center_x, center_y))
                    for xi in np.linspace(center_x + 0.1035, center_x + 0.1379, 3):
                        waypoints.append(make6(robo, arm, xi, 2, center_x, center_y))
                    for xi in np.linspace(center_x + 0.1379, center_x + 0.0688, 3):
                        waypoints.append(make6(robo, arm, xi, 3, center_x, center_y))
                    for xi in np.linspace(center_x + 0.0688, center_x + 0.1035, 3):
                        waypoints.append(make6(robo, arm, xi, 4, center_x, center_y))
                    try:
                        robot = moveit_commander
                        (plan, fraction) = group.compute_cartesian_path(waypoints, 0.0001, 0.0, True)
                        print(fraction)
                        robot = moveit_commander.RobotCommander()
                        plan = group.retime_trajectory(robot.get_current_state(), plan, 1.0)
                        group.execute(plan, wait=True)
                        rospy.sleep(0.1)
                        group.stop()
                    except rospy.ServiceException, e:
                        print "Service call failed: %s"%e
                elif digit == 5:
                    group = MoveGroupCommander(arm + "_arm")
                    group.allow_replanning(True)
                    waypoints =[]
                    for xi in np.linspace(center_y+0.071, center_y, 3):
                        waypoints.append(make5(robo, arm, xi, True, False, center_x, center_y))
                    for xi in np.linspace(center_x, center_x+0.0549, 3):
                        waypoints.append(make5(robo, arm, xi, False, True, center_x, center_y))
                    for xi in np.linspace(center_x+0.0572, center_x+0.1559, 5):
                        waypoints.append(make5(robo, arm, xi, False, False, center_x, center_y))
                    try:
                        robot = moveit_commander
                        (plan, fraction) = group.compute_cartesian_path(waypoints, 0.0001, 0.0, True)
                        print(fraction)
                        robot = moveit_commander.RobotCommander()
                        plan = group.retime_trajectory(robot.get_current_state(), plan, 1.0)
                        group.execute(plan, wait=True)
                        rospy.sleep(0.1)
                        group.stop()
                    except rospy.ServiceException, e:
                        print "Service call failed: %s"%e
                elif digit == 4:
                    group = MoveGroupCommander(arm + "_arm")
                    group.allow_replanning(True)
                    waypoints =[]
                    for xi in range(7):
                        waypoints.append(make4(robo, arm, xi, center_x, center_y))
                    try:
                        robot = moveit_commander
                        (plan, fraction) = group.compute_cartesian_path(waypoints, 0.0001, 0.0, True)
                        print(fraction)
                        robot = moveit_commander.RobotCommander()
                        plan = group.retime_trajectory(robot.get_current_state(), plan, 1.0)
                        group.execute(plan, wait=True)
                        rospy.sleep(0.1)
                        group.stop()
                    except rospy.ServiceException, e:
                        print "Service call failed: %s"%e
                elif digit == 3:
                    group = MoveGroupCommander(arm + "_arm")
                    group.allow_replanning(True)
                    waypoints =[]
                    for xi in np.linspace(center_x-0.0443, center_x+0.0345, 3):
                        waypoints.append(make3(robo, arm, xi, True, center_x, center_y))
                    for xi in np.linspace(center_x+0.0345, center_x+0.0691, 3):
                        waypoints.append(make3(robo, arm, xi, False, center_x, center_y))
                    for xi in np.linspace(center_x+0.0691, center_x+0.1134, 3):
                        waypoints.append(make3(robo, arm, xi, False, center_x, center_y))
                    try:
                        robot = moveit_commander
                        (plan, fraction) = group.compute_cartesian_path(waypoints, 0.0001, 0.0, True)
                        print(fraction)
                        robot = moveit_commander.RobotCommander()
                        plan = group.retime_trajectory(robot.get_current_state(), plan, 1.0)
                        group.execute(plan, wait=True)
                        rospy.sleep(0.1)
                        group.stop()
                    except rospy.ServiceException, e:
                        print "Service call failed: %s"%e
                elif digit == 2:
                    group = MoveGroupCommander(arm + "_arm")
                    group.allow_replanning(True)
                    waypoints =[]
                    for xi in np.linspace(center_x + 0.05, center_x, 3):
                        waypoints.append(make2(robo, arm, xi, True, True, center_x, center_y))
                    for xi in np.linspace(center_x, center_x + 0.05, 3):
                        waypoints.append(make2(robo, arm, xi, True, False, center_x, center_y))
                    for xi in np.linspace(center_x + 0.05, center_x + 0.15, 3):
                        waypoints.append(make2(robo, arm, xi, False, True, center_x, center_y))
                    for xi in np.linspace(center_y - 0.05, center_y + 0.05, 3):
                        waypoints.append(make2(robo, arm, xi, False, False, center_x, center_y))
                    try:
                        robot = moveit_commander
                        (plan, fraction) = group.compute_cartesian_path(waypoints, 0.0001, 0.0, True)
                        print(fraction)
                        robot = moveit_commander.RobotCommander()
                        plan = group.retime_trajectory(robot.get_current_state(), plan, 1.0)
                        group.execute(plan, wait=True)
                        rospy.sleep(0.1)
                        group.stop()
                    except rospy.ServiceException, e:
                        print "Service call failed: %s"%e
                elif digit==1:
                    group = MoveGroupCommander(arm + "_arm")
                    group.allow_replanning(True)
                    waypoints =[]
                    for xi in np.linspace(center_x, center_x+0.1577, 4):
                        waypoints.append(make1(robo, arm, xi, center_x, center_y))
                    try:
                        robot = moveit_commander
                        (plan, fraction) = group.compute_cartesian_path(waypoints, 0.0001, 0.0, True)
                        print(fraction)
                        robot = moveit_commander.RobotCommander()
                        plan = group.retime_trajectory(robot.get_current_state(), plan, 1.0)
                        group.execute(plan, wait=True)
                        rospy.sleep(0.1)
                        group.stop()
                    except rospy.ServiceException, e:
                        print "Service call failed: %s"%e
                elif digit == 0:
                    group = MoveGroupCommander(arm + "_arm")
                    group.allow_replanning(True)
                    waypoints =[]
                    for xi in np.linspace(center_x, center_x + 0.073, 3):
                        waypoints.append(make0(robo, arm, xi, True, center_x, center_y))
                    for xi in np.linspace(center_x + 0.073, center_x+0.146, 3):
                        waypoints.append(make0(robo, arm, xi, True, center_x, center_y))
                    for xi in np.linspace(center_x+0.146, center_x + 0.073, 3):
                        print("new")
                        waypoints.append(make0(robo, arm, xi, False, center_x, center_y))
                    for xi in np.linspace(center_x + 0.073, center_x, 3):
                        waypoints.append(make0(robo, arm, xi, False, center_x, center_y))
                    try:
                        robot = moveit_commander
                        (plan, fraction) = group.compute_cartesian_path(waypoints, 0.0001, 0.0, True)
                        print(fraction)
                        robot = moveit_commander.RobotCommander()
                        plan = group.retime_trajectory(robot.get_current_state(), plan, 1.0)
                        group.execute(plan, wait=True)
                        rospy.sleep(0.1)
                        group.stop()
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
            request.ik_request.pose_stamped.pose.position.x = round(center_x, 3)
            request.ik_request.pose_stamped.pose.position.y = round(center_y, 3)
            request.ik_request.pose_stamped.pose.position.z = -0.05
            request.ik_request.pose_stamped.pose.orientation.x = 0.0
            request.ik_request.pose_stamped.pose.orientation.y = 1.0
            request.ik_request.pose_stamped.pose.orientation.z = 0.0
            request.ik_request.pose_stamped.pose.orientation.w = 0.0
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
                
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e


    except KeyboardInterrupt:
        print('Keyboard Interrupt, exiting')
        break

    # Catch if anything went wrong with the Image Service
    except rospy.ServiceException as e:
        print ("image_process: Service call failed: %s"%e)

cv2.destroyAllWindows()
