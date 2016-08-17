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

## Node Name: img_segm_left/right
## Service Name: img_segm_left/right
## Service Type: Segmentation
## Purpose: Provide a service that returns the latest camera
## sensor data together with its segmentation.

import os
import numpy as np
import cv2
import rospy
from apc2016.srv import CameraSnapshot, CameraSnapshotResponse
from apc2016.srv import Segmentation, SegmentationResponse
import sensor_msgs.msg 
import argparse

from imgproc.learning.segmentation_train import Encoder, Decoder, TDecoder
#from imgproc.learning.segmentation2_train import Encoder, Decoder

import chainer
from chainer import cuda
from chainer import serializers
from chainer import Variable
import chainer.functions as F
import chainer.links as L


from cv_bridge import CvBridge
class ImageSegmentation:
    
    def __init__(self, where, gpu_id):
        self.W = 320
        self.H = 240
        self.gpu_id = gpu_id
        cuda.get_device(gpu_id).use()
        self.enc = Encoder()
        self.dec = Decoder()
        # TODO update to Saito-san's result
        self.tdec = TDecoder()
        my_path = os.path.dirname(os.path.abspath(__file__))
        serializers.load_npz(os.path.join(my_path, "chainer_models/model_enc_160.npz"), self.enc)
        serializers.load_npz(os.path.join(my_path, "chainer_models/model_dec_160.npz"), self.dec)
        serializers.load_npz(os.path.join(my_path, "chainer_models/model_tdec_160.npz"), self.tdec)
        self.dec.to_gpu()
        self.enc.to_gpu()
        self.tdec.to_gpu()
        # TODO wait_for_service() blocks forever if the
        #      service is already there??
        rospy.wait_for_service('snapshot_' + where)
        self.snapshot = rospy.ServiceProxy('snapshot_' + where,
                                           CameraSnapshot)
        self.service = rospy.Service('img_segm_' + where,
                                     Segmentation,
                                     self.segment)
                                     
        self.bridge = CvBridge()
        #self.image_pub = rospy.Publisher("segmentation_image", sensor_msgs.msg.Image, queue_size=1)

    def segment(self, request):
        print request.items
        # make a call to the snapshot service, then
        # run segmentation and return a SegmentationResponse
        snapshot = self.snapshot()
        cuda.get_device(self.gpu_id).use()
        
        a = np.zeros((1, 4, self.H, self.W)).astype(np.float32)
        #print "hoge"
        cv_image = self.bridge.imgmsg_to_cv2(snapshot.realsense_rgb_img, "bgr8")
        cam_rgb = cv2.resize(cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB), (self.W,self.H))
        #cam_rgb = cv2.resize(cv_image, (self.W,self.H))
        #cam_rgb = cv2.flip(cam_rgb, -1)
        cam_rgb = ((cam_rgb - 128.0) / 128.0).transpose(2, 0, 1).astype(np.float32)
        a[0,0:3,:,:] = cam_rgb
        #print cam.img_z.shape
        #print "hoge"
        cv_image = self.bridge.imgmsg_to_cv2(snapshot.realsense_z_img, "mono16")
        cam_z = cv2.resize(cv_image, (self.W, self.H), interpolation=cv2.INTER_NEAREST)
        #cam_z = cv2.flip(cam_z, -1)
        def rescale(x):
            x = (x-30000)/500.0-1
            if x<-1:
                x = -1
            elif x>1:
                x = 1
            return x
        a[0,3,:,:] = np.vectorize(rescale)(np.asarray(cam_z)[:,:].astype(np.float32))
        z = self.enc(Variable(cuda.to_gpu(a), volatile=True), test=True)
        y = self.dec(z, test=True)
        t = self.tdec(z, test=True)
        # y: (1, 40, H, W)
        # TODO consider candidates
        #scores = F.softmax(Variable(y.data, volatile=True)).data.get()
        scores = y.data.get()
        toru = F.softmax(t).data.get()[0,1,:,:]
        # scores: (1, 40, H, W)
        # toru: (H, W)
        #self.image_pub.publish(self.bridge.cv2_to_imgmsg((np.argmax(scores, axis=1)[0,:]).astype(np.uint8), "mono8"))
        
        # build response object
        resp = SegmentationResponse(
            score_map_height=scores.shape[2],
            score_map_width=scores.shape[3],
            score_map=scores.reshape(scores.size).tolist(),
            toru_map=toru.reshape(toru.size).tolist(),
            fx8_pcl_data=snapshot.fx8_pcl_data,
            fx8_x_img=snapshot.fx8_x_img,
            fx8_y_img=snapshot.fx8_y_img,
            fx8_z_img=snapshot.fx8_z_img,
            realsense_pcl_data=snapshot.realsense_pcl_data,
            realsense_rgb_img=snapshot.realsense_rgb_img,
            realsense_x_img=snapshot.realsense_x_img,
            realsense_y_img=snapshot.realsense_y_img,
            realsense_z_img=snapshot.realsense_z_img
        )
        return resp


if __name__=="__main__":
    parser = argparse.ArgumentParser(description="segmentation service")
    parser.add_argument('-w', '--where', required=True, choices=['left','right'], help='left or right')
    parser.add_argument('-g', '--gpu', type=int, default=1, help='number of GPU to use')
    args, unknown = parser.parse_known_args()
    print "ignoring unknown arguments:", unknown
    
    rospy.init_node('img_segm_' + args.where)
    s = ImageSegmentation(args.where, args.gpu)
    rospy.spin()
