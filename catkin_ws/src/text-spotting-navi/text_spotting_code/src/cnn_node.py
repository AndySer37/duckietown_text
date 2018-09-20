#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from std_srvs.srv import Empty
import numpy as np
from os.path import expanduser
import sys
import cv2
from cv_bridge import CvBridge, CvBridgeError
from duckietown_msgs.msg import Rect, Rects, FSMState
#import caffe
import time
np.set_printoptions(threshold=np.inf)
home = expanduser("~")

class CNN_node():
    def __init__(self):
        self.image_sub = rospy.Subscriber("/pbody/image_rect", Image, self.img_cb)
        self.mode_sub = rospy.Subscriber("/pbody/fsm_node/mode", FSMState, self.fsm_mode)
        self.image_pub = rospy.Publisher('gray', Image, queue_size=10)
        self.bridge = CvBridge()
        self.cv_image = 0
        self.cv_img_crop = []
        self.start = 0
        #caffe params
        self.model = model = 'street_en_harvest'
        ##self.caffe_root = home +'/caffe'
        ##sys.path.insert(0, self.caffe_root + 'python')
        ##caffe.set_mode_gpu()
        ##self.net_full_conv = caffe.Net(home+'/models/'+self.model+'.prototxt', home+'/models/'+self.model+'.caffemodel', caffe.TEST)
        nx, ny = (3, 28)
        x = np.linspace(0, 0, nx)
        y = np.linspace(50, 255, ny)
        xv, yv = np.meshgrid(x, y)
        self.class_colors = np.zeros((29,3))
        self.class_colors[0:28]=yv
        self.class_colors[28]=[0,0,0]
        self.class_colors = self.class_colors.astype(np.uint8)
        self.colorful_class_colors = np.random.rand(29,3)*255
        self.colorful_class_colors = self.colorful_class_colors.astype(np.uint8)
        self.switch_quad = 0
        self.switch_img = 1
        self.time = 0
        self.n = 1
        self.stop_line = 0
        
        

    def fsm_mode(self, msg):
        print "FSM node callback"
        if msg.state == "INTERSECTION_CONTROL":
            self.stop_line = 1
            print "start text spotting"        
        elif msg.state == "JOYSTICK_CONTROL":
            self.stop_line = 0
            print "stop text spotting" 
    def img_cb(self, data):
        
        #print "Image callback"
        #print self.stop_line
        #self.switch_img += 1
        #if self.switch_img != 60:
            #print "wait for prediction"
        #    return

        ##if self.stop_line == 0:
        ##    return
        
        try:
            #caffe.set_mode_gpu()
            self.switch_img = 0
            print "switch_on"
            self.start = data.header.stamp.secs
            start = time.time()
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            #np_arr = np.fromstring(data.data, np.uint8)
            #self.cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            img_gray = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)
            mser = cv2.MSER_create(3, 100, 250000, 0.25, 0.2, 200, 1.01, 0.003, 5)
            regions, _ = mser.detectRegions(img_gray)
            hulls = [cv2.convexHull(p.reshape(-1, 1, 2)) for p in regions]
            #cv2.polylines(gray_img, hulls, 1, (255, 0, 0), 2)
            imgContours = self.cv_image
            contour_list = []
            for i, contour in enumerate(hulls):
                x,y,w,h = cv2.boundingRect(contour)
                #repeat = self.checkContourRepeat(contour_list, x, y, w, h)
                #img_region = img_cv[y:y+h, x:x+w]      
                if 2*h < w and h*w < 10000 and h*w > 1000:
                    cv2.rectangle(imgContours,(x, y),(x+w, y+h),(0,255,0),3)
                    img_region = self.cv_image[y:y+h, x:x+w]
                    self.cv_img_crop.append(img_region)
            image_all_mser_image = Image()
            image_all_mser_image.header = rospy.Time.now
            image_all_mser_image = self.bridge.cv2_to_imgmsg(imgContours, "bgr8")
            self.image_pub.publish(image_all_mser_image)
            print "detection time:",time.time()-start
            #self.cnn()  

            
        except CvBridgeError as e:
            print(e)

    def cnn(self):
        if type(self.cv_image) == np.int:
            print "CNN"
            return

        i = 0
        for im in self.cv_img_crop:
        #im = self.cv_image
            start = time.time()
            im = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
            if im is None:
                break
            #im = im/255.0
            #im = cv2.resize(im, (0,0), fx=2, fy=2)
            im = im.reshape(im.shape[0], im.shape[1], 1)
            #print im.shape

            ##transformer = caffe.io.Transformer({'data': self.net_full_conv.blobs['data'].data.shape})
            ##transformer.set_transpose('data', (2,0,1))
            
            #transformer.set_raw_scale('data', 255.0)
            ##self.net_full_conv.blobs['data'].reshape(1,1,32,100)
            ##transformed_image = transformer.preprocess('data', im)
            ##transformed_image -= np.mean(transformed_image)
            #make classification map by forward and print prediction indices at each location
            ##self.net_full_conv.blobs['data'].data[...] = transformed_image
            #self.net_full_conv.blobs['data'].data[...] = im
            #out = self.net_full_conv.forward(data=np.asarray(transformed_image))
            print 'Prediction time:', time.time()-start
            ##out = self.net_full_conv.forward()
            if i == 0:
                now = rospy.get_rostime().secs    
                self.time += (now - self.start)
                print self.n, self.time
                self.n += 1
            i += 1
            
            top1 = out['prob'][0].argmax()
            if out['prob'][0][top1] >= 0.9:
                print 'class: ',top1
                print out['prob'][0][top1][0][0]
                if top1 == 4 or top1 == 2:
                    turn_right = rospy.ServiceProxy('/pbody/open_loop_intersection_control_node/turn_right', Empty)
                    turn = turn_right()
                elif top1 == 23:
                    turn_left = rospy.ServiceProxy('/pbody/open_loop_intersection_control_node/turn_left', Empty)
                    turn = turn_left()
                elif top1 == 14 or top1 == 22:
                    turn_forward = rospy.ServiceProxy('/pbody/open_loop_intersection_control_node/turn_forward', Empty)
                    turn = turn_forward()
                self.stop_line = 0
                break
                print "stop text spotting"
            
    
        self.cv_img_crop = []

    def onShutdown(self):
        rospy.loginfo("[%s] Shutdown." %(self.node_name))  

def main(args):
    rospy.init_node('CNN_node', anonymous = True)        
    ic = CNN_node()
    rospy.on_shutdown(ic.onShutdown)
    rospy.spin() 

if __name__ == '__main__':
    main(sys.argv)
