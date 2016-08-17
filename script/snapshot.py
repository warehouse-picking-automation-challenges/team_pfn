#!/usr/bin/env python

# Copyright 2016 Preferred Networks, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

## Node Name: snapshot_left
## Service Name: snapshot_left
## Service Type: CameraSnapshot
## Subscribed topic:
## - fx8_pcl_data PointCloud2
## - realsense_pcl_data PointCloud2
## - realsense_rgb_img Image
## - fx8_x_img Image
## - fx8_y_img Image
## - fx8_z_img Image
## - realsense_x_img Image
## - realsense_y_img Image
## - realsense_z_img Image
## Purpose: Provide a service that returns the latest received
## frame from each input topics.

import sys
import threading
import rospy
import numpy as np
from sensor_msgs.msg import Image, PointCloud2
from apc2016.srv import CameraSnapshot, CameraSnapshotResponse
from apc2016.msg import XYZImage

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from cv_bridge import CvBridge
import cv2
class fps:
    def __init__(self):
        self.time = rospy.get_time()
        self.lfps = np.zeros((10))
        
    def update(self):
        t = rospy.get_time()
        fps = 1.0/(t - self.time)
        self.lfps = np.roll(self.lfps,1)
        self.lfps[0] = fps
        self.time = t
        
    def strfps(self):
        return '%6.2f' % (np.mean(self.lfps) )



class Snapshot:
    def __init__(self, where):
        rospy.init_node('snapshot_' + where)
        
        self.lock = threading.Lock()
        self.dt = CameraSnapshotResponse()
        
        self.sub = []
        self.sub.append( ( rospy.Subscriber('realsense_pcl_data_' + where,
                                     PointCloud2,
                                     self.callbackRsPcl) , fps() ) )
        self.sub.append( ( rospy.Subscriber('realsense_rgb_img_' + where,
                                     Image,
                                     self.callbackRsRgbImg) , fps() ) )
        self.sub.append( ( rospy.Subscriber('realsense_xyz_img_' + where,
                                     XYZImage,
                                     self.callbackRsXYZimg) , fps() ) )
                                     
        self.sub.append( ( rospy.Subscriber('fx8_pcl_data_' + where,
                                     PointCloud2,
                                     self.callbackFx8Pcl) , fps() ) )
        self.sub.append( ( rospy.Subscriber('fx8_xyz_img_' + where,
                                     XYZImage,
                                     self.callbackFx8XYZimg) , fps() ) )
                                     
                                     
        self.service = rospy.Service('snapshot_' + where,
                                     CameraSnapshot,
                                     self.callbackSrv)

    def callbackSrv(self, request):
        with self.lock:
            return self.dt


    def callbackRsPcl(self, frame):
        with self.lock:
            self.dt.realsense_pcl_data = frame
            self.sub[0][1].update()
        
    def callbackRsRgbImg(self, frame):
        with self.lock:
            self.dt.realsense_rgb_img = frame
            self.sub[1][1].update()
            
    def callbackRsXYZimg(self, frame):
        with self.lock:
            self.dt.realsense_x_img = frame.x
            self.dt.realsense_y_img = frame.y
            self.dt.realsense_z_img = frame.z
            self.sub[2][1].update()
        
    def callbackFx8Pcl(self, frame):
        with self.lock:
            self.dt.fx8_pcl_data = frame
            self.sub[3][1].update()
            
    def callbackFx8XYZimg(self, frame):
        with self.lock:
            self.dt.fx8_x_img = frame.x
            self.dt.fx8_y_img = frame.y
            self.dt.fx8_z_img = frame.z
            self.sub[4][1].update()
            
    @staticmethod
    def save_snapshot(resp, filename, out_image_dir):
        
        dpi = 80
        figsize = 640/dpi, 480/dpi
        fig = plt.figure(figsize=figsize)
        ax = fig.add_axes([0,0,1,1])
        ax.axis('off')
        
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(resp.realsense_rgb_img, "bgr8")
        cam_rgb = cv2.resize(cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB), (640,480))
        
        cv_image = bridge.imgmsg_to_cv2(resp.realsense_x_img, "mono16")
        cam_x = cv2.resize(cv_image, (640,480), interpolation=cv2.INTER_NEAREST)
        cam_x = (np.asarray(cam_x).astype('f')-30000)
        
        cv_image = bridge.imgmsg_to_cv2(resp.realsense_y_img, "mono16")
        cam_y = cv2.resize(cv_image, (640,480), interpolation=cv2.INTER_NEAREST)
        cam_y = (np.asarray(cam_y).astype('f')-30000)
        
        cv_image = bridge.imgmsg_to_cv2(resp.realsense_z_img, "mono16")
        cam_z = cv2.resize(cv_image, (640,480), interpolation=cv2.INTER_NEAREST)
        cam_z = (np.asarray(cam_z).astype('f')-30000)
        
        cv_image = bridge.imgmsg_to_cv2(resp.fx8_x_img, "mono16")
        cam_x2 = cv2.resize(cv_image, (640,480), interpolation=cv2.INTER_NEAREST)
        cam_x2 = (np.asarray(cam_x2).astype('f')-30000)
        
        cv_image = bridge.imgmsg_to_cv2(resp.fx8_y_img, "mono16")
        cam_y2 = cv2.resize(cv_image, (640,480), interpolation=cv2.INTER_NEAREST)
        cam_y2 = (np.asarray(cam_y2).astype('f')-30000)
        
        cv_image = bridge.imgmsg_to_cv2(resp.fx8_z_img, "mono16")
        cam_z2 = cv2.resize(cv_image, (640,480), interpolation=cv2.INTER_NEAREST)
        cam_z2 = (np.asarray(cam_z2).astype('f')-30000)
        
        #plt.rcParams['figure.figsize'] = (6.4,4.8)
        fig.clf()
        ax = fig.add_axes([0,0,1,1])
        ax.axis('off')
        #print np.asarray(cam.img_rgb).shape
        ax.imshow(cam_rgb, interpolation='nearest')
        #plt.axis('off')
#        plt.subplots_adjust(left=0,bottom=0,right=1,top=1)
        ax.set(xlim=[0,640],ylim=[480,0], aspect=1)
        fig.savefig('%s/rgb_%s.png'%(out_image_dir,(filename)), dpi=dpi, transparent=True)
        
        fig.clf()
        ax = fig.add_axes([0,0,1,1])
        ax.axis('off')
        ax.imshow(cam_x, vmin=-500, vmax=500, cmap=plt.cm.gray, interpolation='nearest')
        #ax.axis('off')
        #plt.subplots_adjust(left=0,bottom=0,right=1,top=1)
        ax.set(xlim=[0,640],ylim=[480,0], aspect=1)
        fig.savefig('%s/x_%s.png'%(out_image_dir,(filename)), dpi=dpi, transparent=True)
        
        fig.clf()
        ax = fig.add_axes([0,0,1,1])
        ax.axis('off')
        ax.imshow(cam_y, vmin=-500, vmax=500, cmap=plt.cm.gray, interpolation='nearest')
        #ax.axis('off')
        #plt.subplots_adjust(left=0,bottom=0,right=1,top=1)
        ax.set(xlim=[0,640],ylim=[480,0], aspect=1)
        fig.savefig('%s/y_%s.png'%(out_image_dir,(filename)), dpi=dpi, transparent=True)
        
        fig.clf()
        ax = fig.add_axes([0,0,1,1])
        ax.axis('off')
        ax.imshow(cam_z, vmin=0, vmax=1000, cmap=plt.cm.gray, interpolation='nearest')
        #ax.axis('off')
        #plt.subplots_adjust(left=0,bottom=0,right=1,top=1)
        ax.set(xlim=[0,640],ylim=[480,0], aspect=1)
        fig.savefig('%s/z_%s.png'%(out_image_dir,(filename)), dpi=dpi, transparent=True)
        
        fig.clf()
        ax = fig.add_axes([0,0,1,1])
        ax.axis('off')
        ax.imshow(cam_x2, vmin=-500, vmax=500, cmap=plt.cm.gray, interpolation='nearest')
        #ax.axis('off')
        ax.set(xlim=[0,640],ylim=[480,0], aspect=1)
        fig.savefig('%s/x2_%s.png'%(out_image_dir,(filename)), dpi=dpi, transparent=True)
        
        fig.clf()
        ax = fig.add_axes([0,0,1,1])
        ax.axis('off')
        ax.imshow(cam_y2, vmin=-500, vmax=500, cmap=plt.cm.gray, interpolation='nearest')
        #ax.axis('off')
        ax.set(xlim=[0,640],ylim=[480,0], aspect=1)
        fig.savefig('%s/y2_%s.png'%(out_image_dir,(filename)), dpi=dpi, transparent=True)
        
        fig.clf()
        ax = fig.add_axes([0,0,1,1])
        ax.axis('off')
        ax.imshow(cam_z2, vmin=0, vmax=1000, cmap=plt.cm.gray, interpolation='nearest')
        #ax.axis('off')
        ax.set(xlim=[0,640],ylim=[480,0], aspect=1)
        fig.savefig('%s/z2_%s.png'%(out_image_dir,(filename)), dpi=dpi, transparent=True)


if __name__=="__main__":

    argvs = sys.argv
    if len(argvs) >= 2:
        if argvs[1] == 'left' or argvs[1] == 'right':
            rl = argvs[1]
            s = Snapshot(rl)
        
            tm = rospy.Rate(1)
            while not rospy.is_shutdown():
                
                print "\033[2J"
                for t in s.sub:
                    print('%s %s ' % (t[0].name, t[1].strfps() ) )
                    
                tm.sleep()
                    
    else:
        print('need \'left\' or \'right\' as argument')

    
    
