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

import random

import interface
import util
import shelf_stow_locator
import grab
from grab.vacuum_imgproc import gaussian_interpolation,
import arm_control_wrapper

import conv_bin_coord


(logdebug, loginfo, logwarn, logerr) = util.get_loggers("SmallBinFirstStowing")


class SmallBinFirstStowing(interface.StowProcessStrategy):
    """VisibleGreedyStowing scans the tote and takes the item with the
    highest score to the shelf."""
    def execute(self):
        super(SmallBinFirstStowing, self).calibrate()

        # get a stratey to determine free space
        stow_strategy = shelf_stow_locator.select_strategy(self.pos_info)
        loginfo("ShelfStowLocatorStrategy: %s" % stow_strategy)

        try_count = 0
        items_in_tote = self.pos_info.tote[:]
        while items_in_tote and try_count < 20:
            try_count += 1
            next_item = util.get_smallest_item(items_in_tote)
            loginfo("======================================")
            loginfo("attempt to take item %s from tote" % next_item)

            # first, find a suitable bin
            loginfo("find a location to move that item to")
            (target_bin, target_pos) = \
                stow_strategy.find_location(next_item)
            loginfo("if we succeed to take %s, we will put it to %s/%s" %
                    (next_item, target_bin, target_pos))

            # move pre->front->item_front->item_adj
            arm_control_wrapper.move_left_arm_to_bin(target_bin,'pre')
            (_,_,_,photo_pos) = arm_control_wrapper.move_left_arm_to_bin(target_bin,'photo')
            time.sleep(3.0) # wait for camera delay
            # take snapshot and imgprocs
            # TODO another photo directions
            rospy.loginfo('take snapshot and imgprocs')
            resp = self.segm_service_left(['a'])
            Snapshot.save_snapshot(resp, str(self.log_cnt), 'log_imgs_raw')
            
            target_pos = self.stow_imgproc(bin, resp, photo_pos, next_item)
            
            # back to pre
            rospy.loginfo('back to pre')
            arm_control_wrapper.move_left_arm_to_bin(bin,'pre')
            
            # move the arm back to tote
            try:
                (mv_success, _, _, _) = arm_control_wrapper.move_left_arm_to_bin("tote",
                                                                                 "photo", keitai='nut000', kakujiku='kakujiku')
            except Exception as e:
                logerr("error while moving arm to tote: %s" % e)

            # try all possible grab strategies
            tried_strategies = 0
            for grab_strategy in grab.select_strategies(next_item,
                                                        "tote"):
                # check if this strategy can locate the item
                try:
                    can_grab = grab_strategy.can_grab_item(next_item, "tote",
                                                           items_in_tote)
                except Exception as e:
                    logerr("error while checking for grabability: %s" % e)
                    continue
                if not can_grab:
                    loginfo("strategy %s cannot grab item %s" %
                            (grab_strategy, next_item))
                    continue
                loginfo("can_grab_item was True, try strategy %s for item %s" %
                        (grab_strategy, next_item))

                tried_strategies += 1

                # try to grab the item with this strategy
                try:
                    success = grab_strategy.grab_from_tote(next_item)
                except Exception as e:
                    logerr("error while taking item %s: %s" % (next_item, e))
                    # NB. this is not good, we have no idea what happened here;
                    # the arm is in an undefined state
                    # TODO: need to reset the state somehow
                    continue
                if not success:
                    loginfo("failed to take %s with %s" % (next_item, grab_strategy))
                    continue
                else:
                    loginfo("succeeded to take item %s" % next_item)

                try:
                    success = grab_strategy.stow_in_shelf(target_bin, target_pos)
                except Exception as e:
                    logerr("error while stowing item %s: %s" % (next_item, e))
                    # uh, this is not good, we removed the item from the shelf,
                    # but we failed to put it somewhere.
                    continue
                if not success:
                    # same as exception case before
                    logwarn("we failed to stow item %s in shelf" % next_item)
                    continue

                # if we arrive here, we put the item to the shelf successfully
                self.pos_info.take_from_tote(next_item)
                self.pos_info.put_into_bin(target_bin, next_item)
                items_in_tote.remove(next_item)
                break
            else:
                if tried_strategies > 0:
                    logwarn("we failed to take %s with any strategy" % next_item)
                else:
                    logwarn("we failed to detect %s with any strategy" % next_item)


    # from raw images to strategies
    def stow_imgproc(self, bin, resp, photo_pos, item):
    
        cv_image = self.bridge.imgmsg_to_cv2(resp.realsense_rgb_img, "bgr8")
        cam_rgb = cv2.resize(cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB), (320,240))
        
        fx8_x = (np.asarray(self.bridge.imgmsg_to_cv2(resp.fx8_x_img, "mono16")).astype('f')-30000).reshape((480,640))
        fx8_y = (np.asarray(self.bridge.imgmsg_to_cv2(resp.fx8_y_img, "mono16")).astype('f')-30000).reshape((480,640))
        fx8_z = (np.asarray(self.bridge.imgmsg_to_cv2(resp.fx8_z_img, "mono16")).astype('f')-30000).reshape((480,640))
        fx8_invalid = ((fx8_x==0).astype('i') * (fx8_y==0).astype('i') * (fx8_z==0).astype('i'))
        print fx8_x.shape, fx8_invalid.shape
        xyz_global = ((-1000000*fx8_invalid) + camera_to_global_all(np.asarray([fx8_x,fx8_y,fx8_z]), photo_pos)).astype('f')  # (3, 480, 640) -> (3, 480, 640)
        
        print xyz_global.shape
        
        # depth
        # TODO better downscale
        xyz_global_interpolated = gaussian_interpolation(xyz_global)[:,::2,::2]  # (3, 480, 640) -> (3, 240, 320)
        # normal
        #normal_map = get_normal_image(xyz_global_interpolated)
        
        points = self.img2bin_points(xyz_global_interpolated[0],xyz_global_interpolated[1],xyz_global_interpolated[2], bin )
        
        
        sum_d = self.sum_distace_in_bin( bin , points ); # 6 vector
        space_center = get_space_center( bin , points )
        
        target_pos = space_center
        
        return target_pos
        
    def img2bin_points(self,img_x,img_y,img_z,bin):
        points = []
        for i in range( len(img_x) ):
            for j in range( len(img_x[i]) ):
                point = np.asarray( img_x[i,j] , img_y[i,j] , img_z[i,j] )
                #conv2 bin coord
                self.t_bin_srv = rospy.ServiceProxy('global2bin',CoordinateTransform)
                points.append( twist2array(self.t_bin_srv(bin = bin, point=array2twist(point)).point) )
          
        return np.array( points )  
        
        
    def exchange_large(self,p1,p2):
        for i in range(len(p1)):
            if p1[i] > p2[i] :
                buf = p1[i]
                p1[i] = p2[i]
                p2[i] = buf
        return [p1,p2]
                
    #  how many points in box by p1 p2 
    #  center of points in box
    #  need p1[n] < p2[n]
    def count_in_box(self,box_p1,box_p2,points):
        count = 0
        p = self.exchange_large( box_p1 ,  box_p2)
        for i in range( len(points) ):
            if  p[0][0] < points[i,0] and points[i,0] < p[1][0] \
            and p[0][1] < points[i,1] and points[i,1] < p[1][1] \
            and p[0][2] < points[i,2] and points[i,2] < p[1][2] :
                count+=1
        return count


    def sum_distance_in_box(self,box_p1,box_p2,points):
        count = 0
        sum = [[0,0,0],[0,0,0]]
        p = self.exchange_large( box_p1 ,  box_p2)

        for i in range( len(points) ):
            if  p[0][0] < points[i,0] and points[i,0] < p[1][0] \
            and p[0][1] < points[i,1] and points[i,1] < p[1][1] \
            and p[0][2] < points[i,2] and points[i,2] < p[1][2] :
                count+=1
                sum[0][0] +=  points[i,0] - p[0][0]  
                sum[0][1] +=  points[i,1] - p[0][1] 
                sum[0][2] +=  points[i,2] - p[0][2] 
                sum[1][0] +=  p[1][0] - points[i,0]
                sum[1][1] +=  p[1][1] - points[i,1]
                sum[1][2] +=  p[1][2] - points[i,2]
                
        return sum 
    
    def get_space_center( self,　bin , points ):
        count = 0
        center = np.asarray([0,0,0])
        bin_3side = util.bin_3side[bin]

        for i in range( len(points) ):
            # y is 0 to -bin_3side[1]
            if  0 < points[i,0] and points[i,0] < bin_3side[0] \
            and -bin_3side[1] < points[i,1] and points[i,1] <0 \ 
            and 0 < points[i,2] and points[i,2] < bin_3side[2] :
                count+=1
                # center of x=0 to point[x] 
                center[0] +=  points[i,0] / 2 
                center[1] +=  points[i,1]  
                center[2] +=  points[i,2]  
                 
        return center/count 
    
    
    def sum_distace_in_bin( bin , points):
        p =  [ util.bin_3side[bin][0] , - util.bin_3side[bin][1] , util.bin_3side[bin][2] )
        return self.sum_distance_in_box( [0,0,0] , p , points )
        
       