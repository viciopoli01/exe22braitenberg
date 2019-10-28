#!/usr/bin/env python

import cv2
import numpy as np
import os
import rospy
import yaml

from duckietown import DTROS
from sensor_msgs.msg import CompressedImage
from duckietown_msgs.msg import WheelsCmdStamped
from cv_bridge import CvBridge, CvBridgeError


class BraitenbergNode(DTROS):
    """Braitenberg Behaviour

    This node implements Braitenberg vehicle behavior on a Duckiebot.

    Args:
        node_name (:obj:`str`): a unique, descriptive name for the node
            that ROS will use

    Configuration:
        ~gain (:obj:`float`): scaling factor applied to the desired
            velocity, taken from the robot-specific kinematics
            calibration
        ~trim (:obj:`float`): trimming factor that is typically used
            to offset differences in the behaviour of the left and
            right motors, it is recommended to use a value that results
            in the robot moving in a straight line when forward command
            is given, taken from the robot-specific kinematics calibration
        ~baseline (:obj:`float`): the distance between the two wheels
            of the robot, taken from the robot-specific kinematics
            calibration
        ~radius (:obj:`float`): radius of the wheel, taken from the
            robot-specific kinematics calibration
        ~k (:obj:`float`): motor constant, assumed equal for both
            motors, taken from the robot-specific kinematics calibration
        ~limit (:obj:`float`): limits the final commands sent to the
            motors, taken from the robot-specific kinematics calibration

    Subscriber:
        ~image/compressed (:obj:`CompressedImage`): The acquired camera
            images

    Publisher:
        ~wheels_cmd (:obj:`duckietown_msgs.msg.WheelsCmdStamped`): The
            wheel commands that the motors will execute

    """

    def __init__(self, node_name):

        # Initialize the DTROS parent class
        super(BraitenbergNode, self).__init__(node_name=node_name)
        self.veh_name = rospy.get_namespace().strip("/")
        self.veh=os.environ['VEHICLE_NAME']

        # Use the kinematics calibration for the gain and trim
        self.parameters['~gain'] = None
        self.parameters['~trim'] = None
        self.parameters['~baseline'] = None
        self.parameters['~radius'] = None
        self.parameters['~k'] = None
        self.parameters['~limit'] = None

        # Set parameters using a robot-specific yaml file if such exists
        self.readParamFromFile()
        self.updateParameters()

        # Wait for the automatic gain control
        # of the camera to settle, before we stop it
        rospy.sleep(2.0)
        rospy.set_param('/'+self.veh+'/camera_node/framerate', 15.0)
        rospy.set_param('/'+self.veh+'/camera_node/res_w', 320)
        rospy.set_param('/'+self.veh+'/camera_node/res_h', 240)
        rospy.set_param('/'+self.veh+'/camera_node/exposure_mode', 'off')

        #MyCode
        #Define topics:
        camera_topic="/"+self.veh+"/camera_node/image/compressed"
        wheel_topic="/"+self.veh+"/wheels_driver_node/wheels_cmd"

        #init cv bridge
        self.bridge = CvBridge()
        #Setup the camera subscriber
        self.camera = rospy.Subscriber(camera_topic, CompressedImage, self.imageCallback)
        #Setup the wheel publisher
        self.wheels = rospy.Publisher(wheel_topic, WheelsCmdStamped, queue_size=1)

        #self.pubred = rospy.Publisher("~red/compressed", CompressedImage, queue_size=10)

        self.proportional=4.0

        #BGR COLOR BOUNDARIES
        self.color_boundaries = [
            ([18,18,122],[112,112,252]),
            ([18,122,29],[114,255,106]) # 114 226 106 [186,255,193]
        ]

        self.log("Initialized")

    #
    # MY CODE
    #
    #
    #
    #

    def imageCallback(self,data):
        """called when the image/compressed publish something"""
        
        #convert compressed image to opencv images
        img = self.readIamge(data)
        green = self.extract_color(img,self.color_boundaries[1]) #bgr
        red = self.extract_color(img,self.color_boundaries[0]) #bgr

        #brightness(red)

        """red_image = self.bridge.cv2_to_compressed_imgmsg(
            np.hstack([self.brightness(red),self.brightness(green)]
            )
        )"""

        #self.pubred.publish(red_image)

        #avoiding preval on following
        l,r=self.gotoColor(self.brightness(green))
        self.log("goto go "+("left" if l>r else "right"))
        l,r=self.avoidColor(self.brightness(red))
        self.log("avoid go "+("left" if l>r else "right"))
        l,r = self.speedToCmd(l+1,r+1)
        mess = self.createWheelCmd(l,r)
        self.wheels.publish(mess)

        #rate.sleep()


    def avoidColor(self,img):
        """  """
        white_img=np.sum(img==255)+1
        if white_img<300 : 
            return 0,0

        self.log("area = "+str(white_img))
        white_l=np.sum(img[:,:160]==255)
        white_r=white_img-white_l
        l=(float(white_l)/float(white_img))*self.proportional*2
        r=(float(white_r)/float(white_img))*self.proportional*2
        return l,r

    def gotoColor(self,img):
        """  """
        white_img=np.sum(img==255)+1
        if white_img<100 : 
            return 0,0
        white_l=np.sum(img[:,:160]==255)
        white_r=white_img-white_l
        l=(float(white_l)/float(white_img))*self.proportional
        r=(float(white_r)/float(white_img))*self.proportional
        return l,r

    def extract_color(self,img,color):
        """Return an image with colored area"""
        lower=np.array(color[0],dtype="uint8")
        upper=np.array(color[1],dtype="uint8")

        mask=cv2.inRange(img,lower,upper)
        return cv2.bitwise_and(img,img,mask=mask)


    def readIamge(self,msg_image):
        """Convert images to OpenCV images
            Args:
                msg_image (:obj:`CompressedImage`) the image from the camera node
            Returns:
                OpenCV image
        """
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg_image)
            return cv_image
        except CvBridgeError as e:
            print(e)
            return []
    
    def createWheelCmd(self,left,right):
        wheels_cmd_msg = WheelsCmdStamped()
        # spin right unless servoing or centered
        wheels_cmd_msg.header.stamp = rospy.Time.now()
        wheels_cmd_msg.vel_left = left
        wheels_cmd_msg.vel_right = right
        return wheels_cmd_msg

    def brightness(self,img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (11, 11), 0)
        thresh = cv2.threshold(blurred, 100, 255, cv2.THRESH_BINARY)[1]
        return thresh

    #
    # TEMPLATE CODE
    #
    #
    #
    #

    def speedToCmd(self, speed_l, speed_r):
        """Applies the robot-specific gain and trim to the
        output velocities

        Applies the motor constant k to convert the deisred wheel speeds
        to wheel commands. Additionally, applies the gain and trim from
        the robot-specific kinematics configuration.

        Args:
            speed_l (:obj:`float`): Desired speed for the left
                wheel (e.g between 0 and 1)
            speed_r (:obj:`float`): Desired speed for the right
                wheel (e.g between 0 and 1)

        Returns:
            The respective left and right wheel commands that need to be
                packed in a `WheelsCmdStamped` message

        """

        # assuming same motor constants k for both motors
        k_r = self.parameters['~k']
        k_l = self.parameters['~k']

        # adjusting k by gain and trim
        k_r_inv = (self.parameters['~gain'] + self.parameters['~trim'])\
                  / k_r
        k_l_inv = (self.parameters['~gain'] - self.parameters['~trim'])\
                  / k_l

        # conversion from motor rotation rate to duty cycle
        u_r = speed_r * k_r_inv
        u_l = speed_l * k_l_inv

        # limiting output to limit, which is 1.0 for the duckiebot
        u_r_limited = self.trim(u_r,
                                -self.parameters['~limit'],
                                self.parameters['~limit'])
        u_l_limited = self.trim(u_l,
                                -self.parameters['~limit'],
                                self.parameters['~limit'])

        return u_l_limited, u_r_limited

    def readParamFromFile(self):
        """
        Reads the saved parameters from
        `/data/config/calibrations/kinematics/DUCKIEBOTNAME.yaml` or
        uses the default values if the file doesn't exist. Adjsuts
        the ROS paramaters for the node with the new values.

        """
        # Check file existence
        fname = self.getFilePath(self.veh_name)
        # Use the default values from the config folder if a
        # robot-specific file does not exist.
        if not os.path.isfile(fname):
            self.log("Kinematics calibration file %s does not "
                     "exist! Using the default file." % fname, type='warn')
            fname = self.getFilePath('default')

        with open(fname, 'r') as in_file:
            try:
                yaml_dict = yaml.load(in_file)
            except yaml.YAMLError as exc:
                self.log("YAML syntax error. File: %s fname. Exc: %s"
                         %(fname, exc), type='fatal')
                rospy.signal_shutdown()
                return

        # Set parameters using value in yaml file
        if yaml_dict is None:
            # Empty yaml file
            return
        for param_name in ["gain", "trim", "baseline", "k", "radius", "limit"]:
            param_value = yaml_dict.get(param_name)
            if param_name is not None:
                rospy.set_param("~"+param_name, param_value)
            else:
                # Skip if not defined, use default value instead.
                pass

    def getFilePath(self, name):
        """
        Returns the path to the robot-specific configuration file,
        i.e. `/data/config/calibrations/kinematics/DUCKIEBOTNAME.yaml`.

        Args:
            name (:obj:`str`): the Duckiebot name

        Returns:
            :obj:`str`: the full path to the robot-specific
                calibration file

        """
        cali_file_folder = '/data/config/calibrations/kinematics/'
        cali_file = cali_file_folder + name + ".yaml"
        return cali_file

    def trim(self, value, low, high):
        """
        Trims a value to be between some bounds.

        Args:
            value: the value to be trimmed
            low: the minimum bound
            high: the maximum bound

        Returns:
            the trimmed value
        """

        return max(min(value, high), low)

    def onShutdown(self):
        """Shutdown procedure.

        Publishes a zero velocity command at shutdown."""

        # MAKE SURE THAT THE LAST WHEEL COMMAND YOU PUBLISH IS ZERO,
        # OTHERWISE YOUR DUCKIEBOT WILL CONTINUE MOVING AFTER
        # THE NODE IS STOPPED

        # PUT YOUR CODE HERE
        self.wheels.publish(self.createWheelCmd(0.0,0.0))

        super(BraitenbergNode, self).onShutdown()


if __name__ == '__main__':
    # Initialize the node
    camera_node = BraitenbergNode(node_name='braitenberg')
    # Keep it spinning to keep the node alive
    rospy.spin()
