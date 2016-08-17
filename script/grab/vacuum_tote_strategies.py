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

import grab.vacuum_tote_motions as vacuum_tote_motions
        # TODO orthogonal depth map (for collision avoidance inside bin)
import numpy as np
        
def basic_strategy_tote(bin, item, region, xyz, normal, score_map, toru, photo_dir='center'):
    ret = []
    
    n_regions = region['n_regions']
    for i in range(n_regions):
        mxt = np.max(toru[0,0,:]*region['region'][i,:])
        for jy in range(10,230,2):
            for jx in range(10,310,2):
            
                if region['region'][i,jy,jx]!=1:
                    #print 'not region'
                    continue
                
                if toru[0,0,jy,jx] < 0.3 or toru[0,0,jy,jx] < mxt-0.1:
                    #print 'shoboi2'
                    continue
                if abs(normal[0,jy,jx])<0.7 and abs(normal[1,jy,jx])<0.7 and abs(normal[2,jy,jx])<0.7:
                    #print 'bad normal'
                    #print normal[:,jy,jx]
                    continue
                    
                #print len(ret)
                strategy = {}
                strategy['region'] = region['region'][i,:,:]
                    
                strategy['normal'] = normal[:,jy,jx]
                strategy['px'] = jx
                strategy['py'] = jy
                strategy['pos'] = xyz[:,jy,jx]
                strategy['is_ok'] = True
                strategy['item'] = item
                strategy['toru'] = toru[0,0,jy,jx]
                
                nrm = strategy['normal']
                if abs(nrm[2]) > 0.7:
                    strategy['motion_name'] = 'above'
                    strategy['motion'] = vacuum_tote_motions.pick_tote_from_above(bin, item, strategy['pos'])
                elif abs(nrm[0]) > 0.8 and photo_dir=='xm':
                    strategy['motion_name'] = 'xm'
                    strategy['motion'] = vacuum_tote_motions.pick_tote_from_side(bin, item, strategy['pos'], approach_dir='xm')
                elif abs(nrm[0]) > 0.8 and photo_dir=='xp':
                    strategy['motion_name'] = 'xp'
                    strategy['motion'] = vacuum_tote_motions.pick_tote_from_side(bin, item, strategy['pos'], approach_dir='xp')
                elif abs(nrm[1]) > 0.8 and photo_dir=='ym':
                    strategy['motion_name'] = 'ym'
                    strategy['motion'] = vacuum_tote_motions.pick_tote_from_side(bin, item, strategy['pos'], approach_dir='ym')
                elif abs(nrm[1]) > 0.8 and photo_dir=='yp':
                    strategy['motion_name'] = 'yp'
                    strategy['motion'] = vacuum_tote_motions.pick_tote_from_side(bin, item, strategy['pos'], approach_dir='yp')
                else:
                    strategy['is_ok'] = False
                    strategy['rejected_reason'] = 'invalid direction'
                    #continue
                
                if strategy['is_ok'] and np.sum((normal[:,jy+2,jx]-normal[:,jy,jx])**2) + np.sum((normal[:,jy-2,jx]-normal[:,jy,jx])**2) + np.sum((normal[:,jy,jx+2]-normal[:,jy,jx])**2) + np.sum((normal[:,jy,jx]-normal[:,jy-2,jx])**2) > 0.4:
                    strategy['is_ok'] = False
                    strategy['rejected_reason'] = 'normal instability'
                    
                
                # butsukaru
                if strategy['is_ok'] and strategy['motion'].check_safety() == False:
                    strategy['is_ok'] = False
                    strategy['rejected_reason'] = 'collide with shelf'
                    #continue
                    
                
                ret.append(strategy)
        
    return ret
    
 
def logging_strategy(fp, s, img_name, out_img=True):
    fp.write('is_ok: ' + str(s['is_ok']) + "\n")
    fp.write('item: ' + str(s['item']) + "\n")
    fp.write('pos: ' + str(s['pos']) + "\n")
    fp.write('normal: ' + str(s['normal']) + "\n")
    fp.write('toru: ' + str(s['toru']) + "\n")
    #fp.write('img: ' + str(img_name) + "\n")
    if s['is_ok']:
        fp.write('motion_name: ' + str(s['motion_name']) + "\n")
    elif s.has_key('rejected_reason'):
        fp.write('rejected_reason: ' + str(s['rejected_reason']) + "\n")
