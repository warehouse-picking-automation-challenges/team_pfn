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

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import cv2
import math
import time
import util

import grab.vacuum_motions as vacuum_motions
        # TODO orthogonal depth map (for collision avoidance inside bin)
import numpy as np
        
# 
        
def basic_strategy(bin, item, region, xyz, normal, normal_rs, score_map, toru, photo_dir='center'):
    ret = []
    t_start = time.time()
    n_regions = region['n_regions']
    for i in range(n_regions):
        mxt = np.max(toru[0,0,:]*region['region'][i,:])
        for jy in range(10,230,2):
            for jx in range(10,310,2):
            
                # [water specific] check realsense is valid (this means bottle-label or bottle-cap). other point's depth is broken
                if item=='dasani_water_bottle':
                    if normal_rs[0,jy,jx]==0 or normal_rs[0,jy+4,jx]==0 or normal_rs[0,jy-4,jx]==0 or normal_rs[0,jy,jx+4]==0 or normal_rs[0,jy,jx-4]==0:
                        continue
                
                    
                if region['region'][i,jy,jx]!=1 or region['region'][i,jy+8,jx]!=1 or region['region'][i,jy-8,jx]!=1 or region['region'][i,jy,jx+8]!=1 or region['region'][i,jy,jx-8]!=1:
                    #print 'not region'
                    continue
                    
                if util.item_string_to_volume(item) < 200:
                    toru[0,0,jy,jx] += 0.2
                elif util.item_string_to_volume(item) < 1300:
                    toru[0,0,jy,jx] += 0.1
                else:
                    toru[0,0,jy,jx] -= 0.1
                
                if toru[0,0,jy,jx] < 0.1 or toru[0,0,jy,jx] < mxt-0.3:
                    #print 'shoboi2'
                    continue
                #if abs(normal[0,jy,jx])<0.75 and abs(normal[1,jy,jx])<0.75 and abs(normal[2,jy,jx])<0.75:
                    #print 'bad normal'
                    #print normal[:,jy,jx]
                #    continue
                    
                #print len(ret)
                strategy = {}
                #strategy['region'] = region['region'][i,:,:]
                    
                strategy['normal'] = normal[:,jy,jx]
                strategy['px'] = jx
                strategy['py'] = jy
                strategy['pos'] = xyz[:,jy,jx]
                strategy['is_ok'] = True
                strategy['item'] = item
                strategy['toru'] = toru[0,0,jy,jx]
                
                nrm = strategy['normal']
                if abs(nrm[2]) > 0.75:
                    strategy['motion_name'] = 'above'
                    strategy['motion'] = vacuum_motions.pick_shelf_from_above(bin, item, strategy['pos'])
                elif abs(nrm[0]) > 0.70 and (not item in ['scotch_bubble_mailer', 'hanes_tube_socks', 'kleenex_paper_towels']) :
                    strategy['motion_name'] = 'front'
                    strategy['motion'] = vacuum_motions.pick_shelf_from_front(bin, item, strategy['pos'])
                elif abs(nrm[1]) > 0.8 and photo_dir=='left':
                    strategy['motion_name'] = 'left'
                    strategy['motion'] = vacuum_motions.pick_shelf_from_side(bin, item, strategy['pos'], lr='left')
                elif abs(nrm[1]) > 0.8 and photo_dir=='right':
                    strategy['motion_name'] = 'right'
                    strategy['motion'] = vacuum_motions.pick_shelf_from_side(bin, item, strategy['pos'], lr='right')
                    #continue
                else:
                    strategy['is_ok'] = False
                    strategy['rejected_reason'] = 'invalid direction'
                    #continue
                
                if strategy['is_ok'] and np.sum((normal[:,jy+4,jx]-normal[:,jy,jx])**2) + np.sum((normal[:,jy-4,jx]-normal[:,jy,jx])**2) + np.sum((normal[:,jy,jx+4]-normal[:,jy,jx])**2) + np.sum((normal[:,jy,jx]-normal[:,jy-4,jx])**2) > 1.0:
                    strategy['is_ok'] = False
                    strategy['rejected_reason'] = 'normal instability'
                    
                
                # butsukaru
                if strategy['is_ok'] and strategy['motion'].check_safety() == False:
                    strategy['is_ok'] = False
                    strategy['rejected_reason'] = 'collide with shelf'
                    #continue
                    
                
                ret.append(strategy)
    elapsed_time = time.time() - t_start
    print ("(._.) elapsed_time: {0} [sec]".format(elapsed_time))
    return ret    
    
# centroid above only    
def nuno_strategy(bin, item, region, xyz, normal, normal_rs, score_map, toru, photo_dir='center'):
    ret = []
    
    n_regions = region['n_regions']
    for i in range(n_regions):
        strategy = {}
        print region['region'].shape
        strategy['region'] = region['region'][i,:,:]
        px,py = map(int, region['centroid'][i])
        
        mw = 0.0
        mpx, mpy = 0.0, 0.0
        mpos = np.asarray([0.0, 0.0, 0.0])
        mnrm = np.asarray([0.0, 0.0, 0.0])
        
        for jx in range(int(region['centroid'][i][0]-10),int(region['centroid'][i][0]+11)):
            for jy in range(int(region['centroid'][i][1]-10),int(region['centroid'][i][1]+11)):
                if jx>=0 and jx<toru.shape[3] and jy>=0 and jy<toru.shape[2] and region['region'][i,jy,jx]==1:
                    mw += toru[0,0,jy,jx]
                    mpx += toru[0,0,jy,jx]*jx
                    mpy += toru[0,0,jy,jx]*jy
                    mnrm += toru[0,0,jy,jx]*normal[:,jy,jx]
                    mpos += toru[0,0,jy,jx]*xyz[:,jy,jx]
                
        strategy['px'] = int(mpx/mw)
        strategy['py'] = int(mpy/mw)
        px, py = strategy['px'], strategy['py']
        strategy['pos'] = mpos/mw
        strategy['normal'] = mnrm/mw
        normal_nrm = math.sqrt(np.sum(strategy['normal']**2))
        strategy['normal'] /= normal_nrm
        strategy['is_ok'] = True
        strategy['item'] = item
        strategy['toru'] = toru[0,0,py,px]+0.3
        
        if region['area'][i] < 10:
            strategy['is_ok'] = False
            strategy['rejected_reason'] = 'too small area'
        
        nrm = strategy['normal']
        strategy['motion_name'] = 'above'
        strategy['motion'] = vacuum_motions.pick_shelf_from_above(bin, item, strategy['pos'])
        
        
        # butsukaru
        if strategy['is_ok'] and strategy['motion'].check_safety() == False:
            strategy['is_ok'] = False
            strategy['rejected_reason'] = 'collide with shelf'
        
        # low toru
        #if strategy['is_ok'] and toru[0,0,py,px]<0.1:
        #    strategy['is_ok'] = False
        #    strategy['rejected_reason'] = 'low toru (%f)'%toru[0,0,py,px]
        
        ret.append(strategy)
        
    return ret  
    
    
def basic_strategy_centroid(bin, item, region, xyz, normal,  score_map, toru):
    ret = []
    
    n_regions = region['n_regions']
    for i in range(n_regions):
        strategy = {}
        print region['region'].shape
        strategy['region'] = region['region'][i,:,:]
        px,py = region['centroid'][i]
        
        mw = 0.0
        mpx, mpy = 0.0, 0.0
        mpos = np.asarray([0.0, 0.0, 0.0])
        mnrm = np.asarray([0.0, 0.0, 0.0])
        
        for jx in range(int(region['centroid'][i][0]-10),int(region['centroid'][i][0]+11)):
            for jy in range(int(region['centroid'][i][1]-10),int(region['centroid'][i][1]+11)):
                if jx>=0 and jx<toru.shape[3] and jy>=0 and jy<toru.shape[2] and region['region'][i,jy,jx]==1:
                    mw += toru[0,0,jy,jx]
                    mpx += toru[0,0,jy,jx]*jx
                    mpy += toru[0,0,jy,jx]*jy
                    mnrm += toru[0,0,jy,jx]*normal[:,jy,jx]
                    mpos += toru[0,0,jy,jx]*xyz[:,jy,jx]
                
        strategy['px'] = int(mpx/mw)
        strategy['py'] = int(mpy/mw)
        strategy['pos'] = mpos/mw
        strategy['normal'] = mnrm/mw
        normal_nrm = math.sqrt(np.sum(strategy['normal']**2))
        strategy['normal'] /= normal_nrm
        strategy['is_ok'] = True
        strategy['item'] = item
        
        if region['area'][i] < 10:
            strategy['is_ok'] = False
            strategy['rejected_reason'] = 'too small area'
        
        nrm = strategy['normal']
        if abs(nrm[2]) > 0.7:
            strategy['motion_name'] = 'above'
            strategy['motion'] = vacuum_motions.pick_shelf_from_above(bin, item, strategy['pos'])
        elif abs(nrm[0]) > 0.7:
            strategy['motion_name'] = 'front'
            strategy['motion'] = vacuum_motions.pick_shelf_from_front(bin, item, strategy['pos'])
        elif abs(nrm[1]) > 0.7:
            # TODO
            strategy['is_ok'] = False
            strategy['motion_name'] = 'side'
            strategy['motion'] = vacuum_motions.pick_shelf_from_side(bin, item, strategy['pos'])
            strategy['rejected_reason'] = 'invalid direction (side)'
        else:
            strategy['is_ok'] = False
            strategy['rejected_reason'] = 'invalid direction'
        
        # butsukaru
        if strategy['is_ok'] and strategy['motion'].check_safety() == False:
            strategy['is_ok'] = False
            strategy['rejected_reason'] = 'collide with shelf'
        
        # low toru
        if strategy['is_ok'] and toru[0,0,py,px]<0.1:
            strategy['is_ok'] = False
            strategy['rejected_reason'] = 'low toru (%f)'%toru[0,0,py,px]
        
        ret.append(strategy)
        
    return ret    
     
 
def logging_strategy(fp, s, img_name, out_img=True):
    fp.write('is_ok: ' + str(s['is_ok']) + "\n")
    fp.write('item: ' + str(s['item']) + "\n")
    fp.write('pos: ' + str(s['pos']) + "\n")
    fp.write('normal: ' + str(s['normal']) + "\n")
    fp.write('toru: ' + str(s['toru']) + "\n")
    fp.write('img: ' + str(img_name) + "\n")
    if s['is_ok']:
        fp.write('motion_name: ' + str(s['motion_name']) + "\n")
    elif s.has_key('rejected_reason'):
        fp.write('rejected_reason: ' + str(s['rejected_reason']) + "\n")
