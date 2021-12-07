#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from lab4_cam.srv import ImageSrv, ImageSrvResponse
import cv2, time, sys
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from numpy.linalg import *
import requests
import json
import re
from process_latex import process_sympy
import sympy



# Create a CvBridge to convert ROS messages to OpenCV images
bridge = CvBridge()

# Converts a ROS Image message to a NumPy array to be displayed by OpenCV
def ros_to_np_img(ros_img_msg):
  return np.array(bridge.imgmsg_to_cv2(ros_img_msg,'bgr8'))


if __name__ == '__main__':
  
  # Waits for the image service to become available
  rospy.wait_for_service('last_image')
  
  # Initializes the image processing node
  rospy.init_node('image_processing_node')
  
  # Creates a function used to call the 
  # image capture service: ImageSrv is the service type
  last_image_service = rospy.ServiceProxy('last_image', ImageSrv)


  while not rospy.is_shutdown():
    try:
      # Waits for a key input to continue
    
      raw_input('Press enter to capture an image:')
    except KeyboardInterrupt:
      print 'Break from raw_input'
      break
    
    try:
      # Request the last image from the image service
      # And extract the ROS Image from the ImageSrv service
      # Remember that ImageSrv.image_data was
      # defined to be of type sensor_msgs.msg.Image
      ros_img_msg = last_image_service().image_data

      # Convert the ROS message to a NumPy image
      np_image = ros_to_np_img(ros_img_msg)

    #   # Display the CV Image   
      cv2.imshow("CV Image", np_image)
      api_key = json.load(open('/home/cc/ee106a/fl21/class/ee106a-aeg/ros_workspaces/baxterDraw/src/lab4_cam/src/app_key.json',))
      r = requests.post("https://api.mathpix.com/v3/latex",
      files={"file": cv2.imencode('.jpg', np_image)[1].tobytes()},
      data={"options_json": json.dumps({
          "formats": ["latex_simplified", "asciimath"]})}, headers=api_key)
      print(json.dumps(r.json(), indent=4, sort_keys=True))
      print("__________________________________")
      hopefully_dict = json.loads(json.dumps(r.json(), indent=4, sort_keys=True))


      latex_vals = re.findall(r"'([^']+)'", hopefully_dict['latex_simplified'])

      hopefully_sympy = process_sympy(latex_vals)      

      # When done, get rid of windows and start over
      # cv2.destroyAllWindows()

    except KeyboardInterrupt:
      print 'Keyboard Interrupt, exiting'
      break

    # Catch if anything went wrong with the Image Service
    except rospy.ServiceException, e:
      print "image_process: Service call failed: %s"%e
    
  cv2.destroyAllWindows()