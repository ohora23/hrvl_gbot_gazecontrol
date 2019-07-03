#! /usr/bin/env python
import roslib
import rospy
import tf
import sys
import cv2
import numpy as np
import matplotlib.pyplot as plt
import math

from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PointStamped, Point
from std_msgs.msg import Float64,Float32MultiArray
from sensor_msgs.msg import Image, JointState
from dynamixel_workbench_msgs.srv import *

PI = 3.141592


class PointHeadNode():

    def __init__(self):
        # Initialize new node
        rospy.init_node('pan_tilt_control', anonymous=True)
        rospy.wait_for_service('hrvl_gbot/dynamixel_command')

        dynamixel_namespace = rospy.get_namespace()
        self.rate = rospy.get_param('~rate', 10)
        # r = rospy.Rate(100)
        r = rospy.Rate(30)

        # Initialize the target point
        self.target_point = PointStamped()
        self.last_target_point = PointStamped()

        # tilt(index 1) is attached in reverse direction
        self.ang_dir = [-1, 1]

        # pan/tilt angle
        self.lock = False
        self.c_pt_ang_rad = [0, 0]

        # Subscribe to the target_point topic
        rospy.Subscriber('/target_point', PointStamped,
                         self.update_target_point)
        rospy.Subscriber("hrvl_gbot/joint_states", JointState,
                         self.jstate_callback, queue_size=1)
        
        self.g_pt_arr_pub = rospy.Publisher("hrvl_gbot/gaze_pt_arr",Image,queue_size=1)
        self.suppress_pts_pub = rospy.Publisher("hrvl_gbot/g_suppress",Point,queue_size=1 )
        

        self.head_pan_frame = 'neck_link_z'
        self.head_tilt_frame = 'neck_link_y'

        # gaze_control

        
        self.bridge = CvBridge()
        self.gaze_ang_img = np.zeros((61, 181), dtype=np.uint8)
        # inhibition of return.
        self.IoR_mask = np.zeros_like(self.gaze_ang_img)
        

        # Initialize tf listener
        self.tf = tf.TransformListener()

        # Make sure we can see at least the pan and tilt frames
        self.tf.waitForTransform(self.head_pan_frame, self.head_tilt_frame, rospy.Time(),
                                 rospy.Duration(5.0))  # Wait for maximum 5s.

        # Reset the head position to neutral
        rospy.sleep(1)
        self.reset_head_position()
        rospy.loginfo("Ready to accept target point")
        index = 0

        while not rospy.is_shutdown():
            
            
            self.IoR_mask_time_process()
            self.g_pt_arr_pub.publish(self.bridge.cv2_to_imgmsg(self.IoR_mask,"mono8"))
            

            # rospy.wait_for_message('/target_point', PointStamped)
            # # if self.target_point == self.last_target_point:
            # #     continue
            # try:
            #     target_angles = self.transform_target_point(self.target_point)
            # except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            #     rospy.loginfo("tf Failure")
            #     continue

            # ## use below for testing
            # # send_ang = 10*math.sin(index/10.1*PI/10.1)
            # # self.send_angle_rad(1,send_ang)
            # # self.send_angle_rad(2,send_ang)
            # # rospy.loginfo(str(index)+":ANG:"+str(send_ang))
            # # while(self.lock ==True):

            # rospy.loginfo("cur_pan_tilt:{},{} degs".format(math.degrees(self.c_pt_ang_rad[0]), math.degrees(self.c_pt_ang_rad[1])))
            # rospy.loginfo("PAN:"+str(math.degrees(target_angles[0]))+" TILT:"+str(math.degrees(target_angles[1])))

            # tilt_angle_tmp = target_angles[1]+self.c_pt_ang_rad[1]
            # pan_angle_tmp = target_angles[0]+self.c_pt_ang_rad[0]
            # rospy.loginfo("PAN_final:"+str(math.degrees(pan_angle_tmp))+" TILT_final:"+str(math.degrees(tilt_angle_tmp)))
            # # tilt_angle_tmp = math.radians(10)+self.c_pt_ang_rad[1]
            # # pan_angle_tmp = math.radians(10)+self.c_pt_ang_rad[0]
            # if tilt_angle_tmp > math.radians(30.0):
            #     tilt_angle_tmp = math.radians(30.0)
            # elif tilt_angle_tmp < math.radians(-30.0):
            #     tilt_angle_tmp = math.radians(-30.0)

            # if pan_angle_tmp > math.radians(60.0):
            #     pan_angle_tmp = math.radians(60.0)
            # elif pan_angle_tmp < math.radians(-60.0):
            #     pan_angle_tmp = math.radians(-60.0)
            # ######################
            # i_pan_ang_deg = int(math.degrees(pan_angle_tmp))
            # i_tilt_ang_deg = int(math.degrees(tilt_angle_tmp))

            # d_pt = np.array([21,31])# gaze angle
            # center_pt = np.array([30,90])
            # e_pt = np.array([60,180])

            # # cur_pt = np.array([i_pan_ang_deg, i_tilt_ang_deg])
            # cur_pt = center_pt+np.array([i_tilt_ang_deg, i_pan_ang_deg])
            # c_pt = np.array([])
            # start_pt = cur_pt-d_pt
            # # start_pt[start_pt<0]=0

            # end_pt = cur_pt+d_pt
            # # end_pt[(e_pt-end_pt)<0]= e_pt[(e_pt-end_pt)<0]

            # cv2.rectangle(self.IoR_mask, tuple(start_pt), tuple(end_pt), 255, -1)
            # # cv2.rectangle(self.IoR_mask, (0,0), (2,2), 255, -1)
            # # self.IoR_mask_time_process()

            # ################################################
            # self.send_angle_deg(1,math.degrees(tilt_angle_tmp))# tilt
            # self.send_angle_deg(2,math.degrees(pan_angle_tmp))# pan

            # # self.send_angle_rad(1,target_angles[1])# tilt
            # # self.send_angle_rad(2,target_angles[0])# pan

            # self.last_target_point = self.target_point
            # rospy.loginfo("Setting Target Point:\n" + str(self.target_point))
            # index = index+1

            r.sleep()

    # for Gaze control
    def IoR_mask_time_process(self):
        # self.IoR_mask = self.IoR_mask - 1
        t_ind = self.IoR_mask > 0
        self.IoR_mask[t_ind] = self.IoR_mask[t_ind] -1
        # self.IoR_mask[np.where(self.IoR_mask < 0)] = 0

    def jstate_callback(self, c_jstate):
        self.lock == True
        # self.c_pt_ang_rad[0] = c_jstate.position[1]# PAN
        # self.c_pt_ang_rad[1] = c_jstate.position[0]# TILT
        self.c_pt_ang_rad = [c_jstate.position[1],
            c_jstate.position[0]]  # PAN, tilt

        # rospy.loginfo("cur_pan_tilt:{},{} degs".format(math.degrees(self.c_pt_ang_rad[0]), math.degrees(self.c_pt_ang_rad[1])))
        # print("cur_pan_tilt:{},{} rads".format(self.c_pt_ang_rad[0], self.c_pt_ang_rad[1]))
        self.lock = False

    def update_target_point(self, msg):
        self.target_point = msg

        target_angles = self.transform_target_point(self.target_point)
        
        rospy.loginfo("cur_pan_tilt:{},{} degs".format(math.degrees(
            self.c_pt_ang_rad[0]), math.degrees(self.c_pt_ang_rad[1])))
        rospy.loginfo(
            "PAN:"+str(math.degrees(target_angles[0]))+" TILT:"+str(math.degrees(target_angles[1])))

        tilt_angle_tmp = target_angles[1]+self.c_pt_ang_rad[1]
        pan_angle_tmp = target_angles[0]+self.c_pt_ang_rad[0]
        rospy.loginfo("PAN_final:"+str(math.degrees(pan_angle_tmp))+ \
                        " TILT_final:"+str(math.degrees(tilt_angle_tmp)))
        # tilt_angle_tmp = math.radians(10)+self.c_pt_ang_rad[1]
        # pan_angle_tmp = math.radians(10)+self.c_pt_ang_rad[0]

        ############### Echo back ###########
        suppress_pt = self.transform_pantilt_to_target_point([pan_angle_tmp, tilt_angle_tmp])

        
        self.suppress_pts_pub.publish(suppress_pt.point)

        #####################################
        if tilt_angle_tmp > math.radians(30.0):
            tilt_angle_tmp = math.radians(30.0)
        elif tilt_angle_tmp < math.radians(-30.0):
            tilt_angle_tmp = math.radians(-30.0)

        if pan_angle_tmp > math.radians(60.0):
            pan_angle_tmp = math.radians(60.0)
        elif pan_angle_tmp < math.radians(-60.0):
            pan_angle_tmp = math.radians(-60.0)
        #####################################
        i_pan_ang_deg = int(math.degrees(pan_angle_tmp))
        i_tilt_ang_deg = int(math.degrees(tilt_angle_tmp))

        # d_pt = np.array([20, 30])# gaze angle
        # center_pt = np.array([30, 90])
        # e_pt = np.array([60, 180])
        d_pt = np.array([30, 20])# gaze angle
        center_pt = np.array([90, 30])
        

        cur_pt = center_pt+np.array([i_pan_ang_deg, i_tilt_ang_deg])
        print([i_pan_ang_deg, i_tilt_ang_deg],center_pt)
        
        start_pt = cur_pt-d_pt
        end_pt = cur_pt+d_pt
        


        cv2.rectangle(self.IoR_mask, tuple(start_pt), tuple(end_pt), 255, -1)
        # cv2.rectangle(self.IoR_mask, tuple([start_pt[1],start_pt[0]]), tuple([end_pt[1],end_pt[0]]), 255, -1)
        # cv2.rectangle(self.IoR_mask, (0,0), (2,2), 255, -1)
        self.IoR_mask_time_process()


        ################################################
        self.send_angle_deg(1, math.degrees(tilt_angle_tmp))# tilt
        self.send_angle_deg(2, math.degrees(pan_angle_tmp))# pan


        # self.send_angle_rad(1,target_angles[1])# tilt
        # self.send_angle_rad(2,target_angles[0])# pan

        self.last_target_point = self.target_point
        rospy.loginfo("Setting Target Point:\n" + str(self.target_point))
        

    # Reset with initial parameter setting

    def reset_head_position(self):
        pan_tilt_serv = rospy.ServiceProxy('/hrvl_gbot/dynamixel_command', DynamixelCommand)
        resp1 = pan_tilt_serv("", 2, "Torque_Enable", 0)
        resp2 = pan_tilt_serv("", 2, "Velocity_Limit", 3000)
        resp1 = pan_tilt_serv("", 2, "Torque_Enable", 1)
        resp3 = pan_tilt_serv("", 1, "Goal_Position", 0)
        resp4 = pan_tilt_serv("", 2, "Goal_Position", 0)

        rospy.sleep(1)

    def send_pan_angle(self, pan, tilt):
        pan_tilt_serv = rospy.ServiceProxy('/hrvl_gbot/dynamixel_command', DynamixelCommand)        
        resp = pan_tilt_serv("", 1, "Goal_Position", self.pan)

    def rad2dynamixelM42(self, rad):
        rad_out = rad*151875/PI
        return rad_out

    def deg2dynamixelM42(self, deg):
        deg_out = deg*151875/180
        return deg_out

    def rad2dynamixelL54(self, rad):
        rad_out = rad*180682/PI
        return rad_out

    def deg2dynamixelL54(self, deg):
        deg_out = deg*180692/180
        return deg_out

    def send_angle_rad(self, id, rad_ang):
        pan_tilt_serv = rospy.ServiceProxy('/hrvl_gbot/dynamixel_command', DynamixelCommand)
        try:
            if id ==1:
                resp1 = pan_tilt_serv(
                    "", id, "Goal_Position", self.rad2dynamixelM42(rad_ang))
                return True
            elif id ==2:
                resp2 = pan_tilt_serv(
                    "", id, "Goal_Position", self.rad2dynamixelL54(rad_ang))
                return True
            else:
                print("WRONG_INPUT!")
                return False
        except rospy.ServiceException as e:
            print ("Service call failed:%s" %e)

    def send_angle_deg(self, id, deg_ang):
        pan_tilt_serv = rospy.ServiceProxy('/hrvl_gbot/dynamixel_command', DynamixelCommand)
        try:
            if id ==1:
                resp1 = pan_tilt_serv(
                    "", id, "Goal_Position", self.deg2dynamixelM42(deg_ang))
                return True
            elif id ==2:
                resp2 = pan_tilt_serv(
                    "", id, "Goal_Position", self.deg2dynamixelL54(deg_ang))
                return True
            else:
                print("WRONG_INPUT!")
                return False
        except rospy.ServiceException as e:
            print ("Service call failed:%s" %e)


    def transform_target_point(self, target):
        # Set the pan and tilt reference frames to the head_pan_frame and head_tilt_frame defined above
        pan_ref_frame = self.head_pan_frame
        tilt_ref_frame = self.head_tilt_frame

        # Wait for tf info (time-out in 5 seconds)
        self.tf.waitForTransform(pan_ref_frame, target.header.frame_id, rospy.Time(),
                                 rospy.Duration(5.0))
        self.tf.waitForTransform(tilt_ref_frame, target.header.frame_id, rospy.Time(),
                                 rospy.Duration(5.0))

        # Transform target point to pan reference frame & retrieve the pan angle
        pan_target = self.tf.transformPoint(pan_ref_frame, target)
        print("pan_target ({0},{1},{2})".format(
            pan_target.point.x, pan_target.point.y, pan_target.point.z))
        pan_angle = math.atan2(pan_target.point.y, pan_target.point.x)

        # Transform target point to tilt reference frame & retrieve the tilt angle
        tilt_target = self.tf.transformPoint(tilt_ref_frame, target)
        print("tilt_target ({0},{1},{2})".format(
            tilt_target.point.x, tilt_target.point.y, tilt_target.point.z))
        tilt_angle = math.atan2(tilt_target.point.z,
                                math.sqrt(math.pow(tilt_target.point.x, 2) + math.pow(tilt_target.point.y, 2)))

        # JKYOO: Don't know why this happens... but, have to invert tilt angle
        return [pan_angle, -tilt_angle]

    def transform_pantilt_to_target_point(self, pan_tilt):
        # Set the pan and tilt reference frames to the head_pan_frame and head_tilt_frame defined above
        x_t = 100
        y_t = x_t*math.tan(pan_tilt[1])  # use tilt
        z_t = math.sqrt(x_t*x_t + y_t*y_t) / math.tan(pan_tilt[0])

        tmp_target = PointStamped()
        tmp_target.header.frame_id = self.head_tilt_frame  # tilt frame-based
        tmp_target.point.x = x_t
        tmp_target.point.y = y_t
        tmp_target.point.z = z_t
         

        kinect_ref_frame = "kinect_link"

        self.tf.waitForTransform(kinect_ref_frame, tmp_target.header.frame_id, rospy.Time(),
                                 rospy.Duration(5.0))

        # Transform target point to pan reference frame & retrieve the pan angle
        kinect_target = self.tf.transformPoint(kinect_ref_frame, tmp_target)
        print("kinect_target ({0},{1},{2})".format(
            kinect_target.point.x, kinect_target.point.y, kinect_target.point.z))

        # JKYOO: Don't know why this happens... but, have to invert tilt angle
        return kinect_target


if __name__ == '__main__':
    try:
        point_head = PointHeadNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
