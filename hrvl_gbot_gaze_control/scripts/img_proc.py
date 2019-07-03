#!/usr/bin/python

import roslib
import rospy
import cv2
import sys
import numpy as np
import math
import matplotlib.pyplot as plt
import copy
import timeit
import threading

from std_msgs.msg import String
from geometry_msgs.msg import PointStamped, Point
from sensor_msgs.msg import Image, CameraInfo, JointState
from cv_bridge import CvBridge, CvBridgeError
from IIR2Filter import IIR2Filter

from skimage import data
from skimage.feature import blob_dog, blob_log, blob_doh
from skimage.color import rgb2gray



class GazeControl:

    def __init__(self):
        self.start = True
        # subscribe to kinect image messages
        # qHD = quaterHD(960x540), sd = 512x424, 
        self.qhd_resolution = [960, 540]
        self.sd_resolution = [512, 424]
        self.image_type = 1
        if self.image_type == 1:
            self.sub = rospy.Subscriber("kinect2/qhd/image_mono_rect", Image, self.image_callback, queue_size=1)  
        elif self.image_type == 2:
            self.sub = rospy.Subscriber("kinect2/sd/image_color_rect", Image, self.image_callback, queue_size=1)  

        self.target_pub = rospy.Publisher("target_point", PointStamped, queue_size = 1)

        self.g_pt_arr_pub = rospy.Subscriber("hrvl_gbot/gaze_pt_arr", Image, self.g_pt_arr_callback, queue_size=1)
        
        self.suppress_pts_sub = rospy.Subscriber("hrvl_gbot/g_suppress",Point,self.suppress_pt_callback, queue_size=1 )
        self.suppress_pt = Point()
        self.suppress_pt.x = 0
        self.suppress_pt.y = 0
        self.suppress_pt.z = 0
        # self.sub_jstate = rospy.Subscriber("hrvl_gbot/joint_states", JointState, self.jstate_callback, queue_size=1)
        
        # TODO: Temporary hard coding (have to read from kinect2 camera info (subscribe once))
        self.fxy = [ 515.2761784227258, 515.9218548201856]
        self.cxy = [ 478.23045348775975, 275.136887532946]

      
        self.bridge = CvBridge()

        self.orb = cv2.ORB_create()

        self.cur_image = np.zeros((self.qhd_resolution[1], self.qhd_resolution[0],1), np.uint8)
        self.cur_h_image = np.zeros((self.qhd_resolution[1]/2, self.qhd_resolution[0]/2,1), np.uint8)
        self.cur_q_image = np.zeros((self.qhd_resolution[1]/4, self.qhd_resolution[0]/4,1), np.uint8)
        self.callback_img = np.zeros_like(self.cur_image)
        self.prev_h_image = None #= np.zeros((self.qhd_resolution[1]/4, self.qhd_resolution[0]/2,3), np.uint8)
        self.cur_g_pt_image = np.zeros((61, 181),np.uint8)

        # gaze_control
        self.gaze_ang_img = np.zeros((180,60), dtype=np.uint8)
        self.IoR_mask = np.zeros_like(self.gaze_ang_img) # inhibition of return.

        ''' Optical flow '''
        
        self.color = np.random.randint(0,255,(2000,3))
        # self.lines = None
        self.termcriteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03)
        self.prevPt = None
        self.curPt = None
        self.curPt_filtered = None
        self.curMv = None
        
        # Image Matching
        self.drawing_offset = 10 # drawing offset
        self.filling_mask = None

        self.key_bmargin = 5
        # Blur detection
        self.blurry_max = 2500
        

        self.pause_subscribing = False

        self.debug_window = 'MouseCallback'
        cv2.imshow(self.debug_window, self.cur_image)
        cv2.setMouseCallback(self.debug_window, self.onMouse)
        
        ## IIR Filter
        self.fs = 1000
        # self.x_filter = IIR2Filter(3, e[50.0],'lowpass', fs=self.fs)
        # self.y_filter = IIR2Filter(3, [50.0],'lowpass', fs=self.fs)
        self.x_filter = IIR2Filter(3, [20.0],'lowpass', fs=self.fs)
        self.y_filter = IIR2Filter(3, [20.0],'lowpass', fs=self.fs)


        # PanTilt control
        self.c_index = 0
        self.c_index_max = 6
        self.init_count = 0
        self.init_count_max = 1000
    #     # pan/tilt angle
    #     self.lock = False
    #     self.c_pt_ang_rad = [0,0]
        
    # def jstate_callback(self, c_jstate):
    #     self.lock = True
    #     self.c_pt_ang_rad[0] = c_jstate.position[1]
    #     self.c_pt_ang_rad[1] = c_jstate.position[0]
    #     print("cur_pan_tilt:{},{} degs".format(math.degrees(self.c_pt_ang_rad[0]), math.degrees(self.c_pt_ang_rad[1])))


    #     self.lock = False

        # for Gaze control
    def IoR_mask_time_process(self, delta_t):
        self.IoR_mask = self.IoR_mask -1
        self.IoR_mask[np.where(self.IoR_mask < 0)] = 0

    # def pan_tilt_visible_area_marking(self, g_dir, eval):
    #     # at here, use box filter for pan/tilt angles

    def suppress_pt_callback(self, pt):
        self.suppress_pt = pt
    
    def variance_of_laplacian(self, image):
    	# compute the Laplacian of the image and then return the focus
	    # measure, which is simply the variance of the Laplacian
	    return cv2.Laplacian(image, cv2.CV_64F).var()

    def g_pt_arr_callback(self, image):
        self.cur_g_pt_image = self.bridge.imgmsg_to_cv2(image)
        

    def image_callback(self, data): 
        start = timeit.default_timer()
        
        if self.pause_subscribing == False:
            try:

                self.cur_image = self.bridge.imgmsg_to_cv2(data)
                self.cur_h_image= cv2.resize(self.cur_image, None,fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)
                self.cur_q_image= cv2.resize(self.cur_image, None,fx=0.25, fy=0.25, interpolation=cv2.INTER_AREA)

                # kp1, desc1 = self.orb.detectAndCompute(self.prev_h_image, None)
                kp2, desc2 = self.orb.detectAndCompute(self.cur_h_image, None)
                
                # matcher = cv2.BFMatcher(cv2.NORM_HAMMING2)
                # matches = matcher.knnMatch(desc1, desc2, 2)

                # good_matches = [first for first, second in matches \
                #                 if first.distance < second.distance * self.ratio]
                # rospy.loginfo("GoodMatches:{}/{}".format(len(good_matches), len(matches)))

                # for kp in kp1:
                #     print("kps:{}".format(kp.pt))

                ##############################################
                # Blur check
                fm=self.variance_of_laplacian(self.cur_h_image)
                blur_index = ((self.blurry_max - min(fm, self.blurry_max)) / self.blurry_max)*100;
                
                if blur_index > 50:
                    rospy.loginfo("BLURRY:{} percent".format(blur_index))
                else:
                    rospy.loginfo("_:{} percent".format(blur_index))
                ##############################################

                img_draw = self.cur_h_image.copy()
                
                
                


                img_draw_matches = self.cur_h_image.copy()
                # img_draw = cv2.drawKeypoints(self.cur_h_image, keypoints, None,\
                #     flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
                
                diff_image = np.zeros_like(self.cur_h_image)
                diff_image_q = np.zeros_like(self.cur_q_image)

                # Optical flow
                if self.start == True:
                    self.start = False
                    rospy.loginfo("[Mid-gaze emergency-detection module] : Started ")
                    self.prev_h_image = self.cur_h_image
                    feature_image = self.cur_h_image
                    self.lines = np.zeros_like(self.cur_h_image)
                    self.prevPt = cv2.goodFeaturesToTrack(feature_image, 500, 0.01, 5, 14)

                else:
                    h,w = self.cur_h_image.shape[:2] # basic image width/height

                    feature_image_prev = self.prev_h_image
                    feature_image_cur = self.cur_h_image
                    self.lines = np.zeros_like(self.cur_h_image)
                    self.prevPt = cv2.goodFeaturesToTrack(feature_image_prev, 100, 0.01, 5, 14)
                    # stop_criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

                    # cv2.cornerSubPix(feature_image_prev, self.prevPt, (11, 11), (-1, -1), stop_criteria)

                    
                    curPt_raw, status, err = cv2.calcOpticalFlowPyrLK(feature_image_prev, feature_image_cur, \
                        self.prevPt, None, criteria = self.termcriteria)
        
                        
                
                    ## TOO SLOW
                    # cv2.cornerSubPix(feature_image_cur, curPt_raw, (11, 11), (-1, -1), stop_criteria)

                    for i, p in enumerate(self.prevPt):
                        px, py = p.ravel()
                        if (px < self.key_bmargin) | (px > w-self.key_bmargin):
                            status[i] = 0    
                        elif (py < self.key_bmargin+30) | (py > h-self.key_bmargin):
                            status[i] = 0
                    
                    
                    
                    for i, p in enumerate(curPt_raw):
                        px, py = p.ravel()
                        if (px < self.key_bmargin) | (px > w-self.key_bmargin):
                            status[i] = 0   
                            
                        elif (py < self.key_bmargin+30) | (py > h-self.key_bmargin):
                            status[i] = 0
                            
                
        
                    self.prevMv = self.prevPt[status == 1]
                    self.curMv = curPt_raw[status == 1]
                    
                    # ## Just for visualizing image
                    # for i, (p, n) in enumerate(zip(self.prevMv, self.curMv)):
                    #     px, py = p.ravel()
                    #     nx, ny = n.ravel()
                    #     cv2.line(self.lines, (px, py), (nx,ny), self.color[i].tolist(), 2)
                    #     cv2.circle(img_draw, (nx, ny), 2, self.color[i].tolist(), -1)


                    H_matrix, mask = cv2.findHomography(self.prevMv, self.curMv, cv2.RANSAC, 10.0)
                    
                    
                    moving_img_q = np.zeros_like(self.cur_q_image)
                    out_boxfilter_img = np.zeros_like(self.cur_q_image)

                    num_outlier = 0
                    for i, (p, n) in enumerate(zip(self.prevMv, self.curMv)):
                        # if mask[i] == 0: # outlier
                            px, py = p.ravel()
                            nx, ny = n.ravel()
                            cv2.line(self.lines, (px, py), (nx,ny), self.color[i].tolist(), 2)
                            cv2.circle(img_draw, (nx, ny), 2, self.color[i].tolist(), -1)
                            moving_img_q[py/2, px/2] = 255
                            num_outlier=num_outlier+1
                    if num_outlier > 0:
                        # out_boxfilter_img = cv2.boxFilter(moving_img_q, -1, (30, 30))
                        out_boxfilter_img = cv2.boxFilter(moving_img_q, -1, (20, 20))
                        minVal, maxVal, minLoc, maxLoc = cv2.minMaxLoc(out_boxfilter_img)
                        out_boxfilter_img = cv2.equalizeHist(out_boxfilter_img)

                        # LPF for maxLoc
                        filtered_x = self.x_filter.filter(maxLoc[0])
                        filtered_y = self.y_filter.filter(maxLoc[1])

                      

                        # blobs_dog = blob_dog(out_boxfilter_img, max_sigma=15, threshold=.1)
                        # blobs_dog[:, 2] = blobs_dog[:, 2] * math.sqrt(2)
                        
                        # for blob in blobs_dog:
                        #     yt,xt,r = blob
                        #     if r > 30:
                        #         cv2.circle(out_boxfilter_img, (int(xt), int(yt)), int(r), 0, -1)


                        #cv2.circle(out_boxfilter_img, maxLoc, 2, 0, -1)
                        cv2.circle(out_boxfilter_img, (int(filtered_x), int(filtered_y)), 5, 50, -1)
                        # suppress_pt
                        sup_img_pt = self.convert_target3D_to_point(self.suppress_pt)
                        cv2.circle(out_boxfilter_img, tuple([int(sup_img_pt[0]),int(sup_img_pt[1])]), 2, 0, -1)    
                        print("SuppressPT:{}".format(sup_img_pt))

                        cv2.imshow("Image window(BOXfiltered)", out_boxfilter_img)


                        img_c_x = int(w/8);
                        img_c_y = int(h/8);

                        if self.init_count > self.init_count_max: # initial delay
                            if (self.c_index > self.c_index_max):
                                if (abs(img_c_x - filtered_x) >20) | (abs(img_c_y - filtered_y)>20):
                                    self.publish_img2Dpt(np.array([filtered_x, filtered_y])*4)
                                self.c_index = 0
                            else:
                                self.c_index = self.c_index+1
                        else:
                            rospy.loginfo("Countdown:{}".format(self.init_count_max - self.init_count))
                            self.init_count = self.init_count+1


                    
                    
                    
                    
                    do = self.drawing_offset
                    # pts = np.float32([ [[0,0]], [[0,h-1]],[[w-1,h-1]],[[w-1,0]] ])
                    pts = np.float32([ [[do,do]], [[do,h-1-do]],[[w-1-do,h-1-do]],[[w-1-do,do]] ]) # assign margines for drawing
                    
                    dst = cv2.perspectiveTransform(pts, H_matrix)
                    img_draw_matches = cv2.polylines(img_draw_matches, [np.int32(dst)], True, 255, 3, cv2.LINE_AA)
                    # generated mask
                    self.filling_mask = np.zeros_like(self.cur_h_image)
                    cv2.fillConvexPoly(self.filling_mask, np.int32(dst), (255, 255, 255))

                    
                    overlay = cv2.warpPerspective(self.prev_h_image, H_matrix, (w,h))

                    # Not necessary
                    # img_draw_matches = cv2.addWeighted(img_draw_matches, 0.5, overlay, 0.5, 0.0)

                    # blurring for two images
                    # blur_cur_h_image = cv2.medianBlur(self.cur_h_image, 5) # use odd # of kernel size
                    # blur_overlay = cv2.medianBlur(overlay,5)

                    # blur_cur_h_image = cv2.GaussianBlur(self.cur_h_image, (9,9),0) # use odd # of kernel size
                    # blur_overlay = cv2.GaussianBlur(overlay,(9,9),0)


                    diff_image = cv2.absdiff(self.cur_h_image, overlay)
                    # diff_image = cv2.absdiff(blur_cur_h_image, blur_overlay)
        
                    diff_image_masked = cv2.bitwise_and(diff_image, self.filling_mask)
                    ret, diff_image = cv2.threshold(diff_image_masked,20,255,cv2.THRESH_BINARY)
                    
                    # erosion
                    kernel = cv2.getStructuringElement(cv2.MORPH_CROSS, (3,3))

                    diff_image_q = cv2.pyrDown(diff_image)

                
                    

                    diff_image_q = cv2.erode(diff_image_q, kernel,iterations=7)
                    diff_image_q = cv2.dilate(diff_image_q, kernel,iterations=7)
                    
                    # Moving Object Detection
                    im, contours, hr = cv2.findContours(diff_image_q,cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    for contr in contours:
                        # x,y,w,h = cv2.boundingRect(contr)
                        (x,y),radius = cv2.minEnclosingCircle(contr)
                        if radius > 30:
                            cv2.circle(diff_image_q, (int(x), int(y)), int(radius), 255, 2)




                    # img_draw = cv2.add(img_draw, self.lines)



                    self.prev_h_image = self.cur_h_image
                    # self.prevPt = curPt_raw(status_curpt == 1)
                    # self.prevPt = self.curPt.reshape(-1, 1, 2)

            except CvBridgeError as e:
               print(e)

            # cv2.imshow("Image window(half)", self.cur_h_image)
            # cv2.imshow("Image window(half_features)_Features", img_draw)


        
            cv2.imshow("Image window(half_features_matches)", img_draw_matches)
            cv2.imshow("Image window(diff)", diff_image_q)
            cv2.imshow("Image window(outliers):Features", img_draw)
            cv2.imshow("CurPT", self.cur_g_pt_image)

        stop = timeit.default_timer()
        print("Elapsed time: {}ms".format((stop-start)*1000))

        # cv2.imshow(self.debug_window, self.callback_img)
        cv2.imshow(self.debug_window, self.cur_image)
        cv2.waitKey(3)


    def publish_img2Dpt(self, cur_pt):
        out_pt = self.convert_point_to_target3D(cur_pt, 10) # convert to original image coordinate
        # rospy.loginfo("[GC]TargetPoint3D : ({},{},{})".format(out_pt.point.x, out_pt.point.y, out_pt.point.z))
        self.target_pub.publish(out_pt)


    def onMouse(self, event, x, y, flags, param):
        # self.pause_subscribing = True
        # print(event, x, y,)
        if event == cv2.EVENT_LBUTTONDOWN:
            cv2.circle(self.callback_img, (x,y), 3, 255, -1)
            out_pt = self.convert_point_to_target3D([x, y], 10)
            rospy.loginfo("[GC]TargetPoint3D : ({},{},{})".format(out_pt.point.x, out_pt.point.y, out_pt.point.z))
            self.target_pub.publish(out_pt)

    # Convert to normalized coordinate
    def convert_point_to_target3D(self, img_pt, depth): # qhd resolution-based

        target_pt = PointStamped()
        target_pt.header.frame_id = "kinect_link"
        target_pt.point.x = depth * (img_pt[0]-self.cxy[0])/self.fxy[0] # X calc
        target_pt.point.y = depth * (img_pt[1]-self.cxy[1])/self.fxy[1] # Y calc
        target_pt.point.z = depth

        return target_pt
        
    def convert_target3D_to_point(self, t_pt): # qhd resolution-based
        img_pt = [0,0]
        print("sup_t_pt:{}".format(t_pt))
        if t_pt.z > 0:
            img_pt[0] = (t_pt.x*self.fxy[0])/t_pt.z+self.cxy[0]
            img_pt[1] = (t_pt.y*self.fxy[1])/t_pt.z+self.cxy[1]
            return img_pt
        else:
            return [-1,-1]

    def image_callback_sd(self, data):
        try:
            self.cur_image_sd = self.bridge.imgmsg_to_cv2(data)
            self.cur_h_image_sd= cv2.resize(self.cur_image_sd, None,fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)

            # x,y,w,h = cv2.selectROI('Image_sd(half)',self.cur_h_image_sd, False)

            keypoints, descriptor = self.orb.detectAndCompute(self.cur_h_image_sd, None)
            img_draw = cv2.drawKeypoints(self.cur_h_image_sd, keypoints, None,\
                flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)



        except CvBridgeError as e:
            print(e)
        cv2.imshow("Image_sd(orig)", self.cur_image_sd)
        cv2.imshow("Image_sd(half)", self.cur_h_image_sd)
        cv2.imshow("Image_sd(half_features)", img_draw)
        cv2.waitKey(3)

def main(args):
    ic = GazeControl()
    rospy.init_node('gaze_img_proc')

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)