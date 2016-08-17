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

import numba
import numpy as np

import cv2

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

@numba.jit('void(f4[:,:,:], i8, f4, f4, f4[:,:,:])', nopython=True)
def _gaussian_interpolation(src, ksize, threshold, beta, dst):
    rows, cols = src.shape[1], src.shape[2]
    K = ksize // 2

    kernel_table = np.empty((ksize, ksize), dtype=np.float32)
    for i in range(ksize):
        for j in range(ksize):
            dx, dy = i - K, j - K
            d2 = dx * dx + dy * dy
            kernel_table[i, j] = np.exp(- beta * d2)

    for y in range(K,rows-K):
        for x in range(K,cols-K):
            val = np.zeros(3)
            weight_sum = 0.0
            flag = False
            for dy in range(-K, K + 1):
                for dx in range(-K, K + 1):
                    u, v = x + dx, y + dy
                    #if (0 <= u < cols) and (0 <= v < rows):
                    if -10000<src[2, v, u] :  # invalid point is z==-100000 (tabun)
                        #d2 = dx * dx + dy * dy
                        #w = np.exp(- beta * d2)
                        w = kernel_table[dy + K, dx + K]
                        val += w * src[:, v, u]
                        weight_sum += w
                        flag = True
            if flag:
                dst[:, y, x] = (val / weight_sum)
            else:
                dst[:, y, x] = src[:, y, x]

def gaussian_interpolation(src, ksize=31, threshold=1.0, beta=0.014):
    assert(src.ndim == 3)
    assert(ksize % 2 == 1)
    dst = np.zeros_like(src)
    _gaussian_interpolation(src, ksize, threshold, beta, dst)
    return dst



@numba.jit
def normalized_cross(vec1, vec2):
    a1, a2, a3 = vec1[0], vec1[1], vec1[2]
    b1, b2, b3 = vec2[0], vec2[1], vec2[2]
    result = np.zeros(3)
    result[0] = a2 * b3 - a3 * b2
    result[1] = a3 * b1 - a1 * b3
    result[2] = a1 * b2 - a2 * b1
    #print result[0]
    nrm = np.sqrt(result[0]**2 + result[1]**2 + result[2]**2)
    return result/nrm


# input: gaussian normalized xyz image
@numba.jit('f4[:,:,:](f4[:,:,:])', nopython=True)
def _get_normal_image(src):
    rows, cols = src.shape[1], src.shape[2]
    dst = np.zeros_like(src)
    xgrad = np.zeros_like(src)
    ygrad = np.zeros_like(src)
    for y in range(2, rows-2):
        for x in range(2, cols-2):
            #xgrad[:,y, x] = (1.0/2.0)*(src[:,y-1, x+1] - src[:,y-1, x-1])/4.0
            xgrad[:,y, x] = (1.0/2.0)*(src[:,y, x+1] - src[:,y, x-1])
            #xgrad[:,y, x] += (1.0/2.0)*(src[:,y+1, x+1] - src[:,y+1, x-1])/4.0
    for y in range(2, rows-2):
        for x in range(2, cols-2):
            #ygrad[:,y, x] = (1.0/2.0)*(src[:,y+1, x-1] - src[:,y-1, x-1]) /4.0
            ygrad[:,y, x] = (1.0/2.0)*(src[:,y+1, x] - src[:,y-1, x])
            #ygrad[:,y, x] += (1.0/2.0)*(src[:,y+1, x+1] - src[:,y-1, x+1]) /4.0
    for y in range(rows):
        for x in range(cols):
            dst[:,y,x] = normalized_cross(xgrad[:,y,x], ygrad[:,y,x])
    return dst

def get_normal_image(src):
    assert(src.ndim == 3)
    return _get_normal_image(src)


def decompose_into_regions(argmax_img, item_dict=None):
    elements = np.unique(argmax_img)
    label_image = np.empty_like(argmax_img, dtype=np.uint8)
    result = {}
    for index in elements:
        if index == 0:
            # skip background label
            continue
        label_image[:] = 0
        label_image[argmax_img == index] = 1
        n_regions, region, bbs, centroid = \
            cv2.connectedComponentsWithStats(label_image, ltype=cv2.CV_16U)
        key = index if item_dict is None else item_dict[index]
        
        if n_regions==1:
            continue
        
        idx = []
        regions = []
        for i in range(1, n_regions):
            if bbs[i,4]>50:
                idx.append(i)
                regions.append(region==i)
                
        n_regions_filtered = len(idx)
            
        print n_regions, n_regions_filtered, region.shape, region[1:].shape
        if n_regions_filtered>0:
            result[key] = {
                "item_id": index,
                "n_regions": n_regions_filtered, # ignore background region
                "region": np.asarray(regions).astype('i'),
                "bounding_box": bbs[idx, :4],
                "area": bbs[idx, 4],
                "centroid": centroid[idx],
            }
    return result


class VacImgProc:
    def __init__(self):
        self.threshold_score = 0.6
        self.threshold_toru = 0.2
        self.threshold_nrm = 0.8
        
        
        
    # get housen
    #   PCA -> 3rd pricipal component
    def get_normal(self, ps):
        m = np.mean(ps, axis=0, keepdims=True)
        #print m
        p_m = ps - m
        l, v = np.linalg.eig(np.dot(p_m.T, p_m))
        p2 = v[:,2] / np.linalg.norm(v[:,2])
        od = np.argsort(l)
        #print l, od
        p2 = v[:,od[0]]
        if p2[2]<0:
            p2 *= -1.0
        #print "normal", p2
        return p2, l[od[0]]
        
    # get candidate approach for each items
    @staticmethod
    def get_candidates(self, score_map, toru, items_in_bin, str_map, reject_map):
        # get rid of items in other bin
        tmp = []
        if len(items_in_bin)>0:
            for i in items_in_bin:
                item_id = item_string_to_id(i)
                if item_id in tmp:
                    continue
                tmp.append(item_id)
                #print item_id, i
                #item_id = i
                score_map[0,item_id,:,:] += 10
            score_map[0,1:,:] -= 10
            
        # avepool for noise robustness
        aprob = F.average_pooling_2d(Variable(score_map.astype(np.float32)), ksize=11, stride=1, pad=5)
        aprob = F.average_pooling_2d(aprob, ksize=11, stride=1, pad=5)
        aprob = F.average_pooling_2d(aprob, ksize=11, stride=1, pad=5)
        aprob = F.softmax(Variable(aprob.data.astype(np.float32), volatile=True))
        aprob = aprob.data
        prob = F.softmax(Variable(score_map.astype(np.float32), volatile=True))
        
        # for each pixel:
        #  score > ts
        #  toru > tt
        #  normal & item_id -> strategy
        #  strategy -> avoid collision
        candidates = [[] for i in range(40)]
        for i in range(240):
            for j in range(320):
                if toru[0,0,i,j] < self.threshold_toru:
                    reject_map[i,j] = 2 # low toru score
                    continue
                item_id = np.argmax(score_map[0,:,i,j])
                if aprob[0,item_id,i,j] < self.threshold_score:
                    reject_map[i,j] = 3 # low segmentation score
                    continue
                if item_id==0:
                    reject_map[i,j] = 4 # this is background
                    continue
                candidate = {}
                candidata['item_id'] = item_id
                candidate['confidence_score'] = aprob[0,item_id, i, j]
                candidate['confidence_toru'] = toru[0,0,i,j]
                candidate['pixel_x'] = j
                candidate['pixel_y'] = i
                candidates[item_id].append(candidate)
        return candidates
    
    # check normal vector and decide approach strategy in candidates
    # xyz: depth map in global coordinate
    @staticmethod
    def normal_check(self, candidates, bin, img_x, img_y, img_z, str_map, reject_map):
        ret = []
        str_map *= 0
        for i in range(0,40):
            ret.append([])
            for c in candidates[i]:
                ix,iy,iz = img_x[c['pixel_y'],c['pixel_x']], img_y[c['pixel_y'],c['pixel_x']], img_z[c['pixel_y'],c['pixel_x']]
                inside = LeftRight_arm.inside_bin_check([ix,iy,iz+75], bin)
                if not inside:
                    reject_map[c['pixel_y'],c['pixel_x']] = 5 # outside bin
                    continue
                # get points around target position
                ps = []
                tmp = 0
                for ii in range(-14,15,1):
                    for jj in range(-14,15,1):
                        if img_x[c['pixel_y']+ii,c['pixel_x']+jj]<100:
                            continue
                        else:
                            ps.append([img_x[c['pixel_y']+ii,c['pixel_x']+jj], img_y[c['pixel_y']+ii,c['pixel_x']+jj], img_z[c['pixel_y']+ii,c['pixel_x']+jj]])
                        #print "# ", ps[-1]
                mps = np.mean(np.asarray(ps), axis=0)
                ps2 = []
                for p in ps:
                    # remove hazure
                    if np.sum((p-mps)**2) < 300:
                        ps2.append(p)
                    #else:
                    #    print "hoge"
                
                if len(ps2)<5:
                    #print "too few valid points"
                    reject_map[c['pixel_y'],c['pixel_x']] = 6 # normal estimation failed
                    continue
                    
                nrm, ln = self.get_normal(np.asarray(ps2))
                
                #print ln
                #if ln>100:
                #    continue
                    
                if abs(nrm[2]) > 0.8:
                    str_map[c['pixel_y'],c['pixel_x']] = 1
                    c['approach_strategy'] = 'above'
                elif abs(nrm[0]) > 0.85:
                    str_map[c['pixel_y'],c['pixel_x']] = 2
                    c['approach_strategy'] = 'front'
                elif (nrm[1]) > 0.85:
                    # TODO left/right
                    str_map[c['pixel_y'],c['pixel_x']] = 3
                    c['approach_strategy'] = 'left'
                elif (nrm[1]) < -0.85:
                    # TODO left/right
                    str_map[c['pixel_y'],c['pixel_x']] = 4
                    c['approach_strategy'] = 'right'
                else:
                    reject_map[c['pixel_y'],c['pixel_x']] = 7 # cannot approach because of normal vector direction
                    continue
                
                #print c['approach_strategy']
                if c['approach_strategy']=='front':
                    inside = LeftRight_arm.inside_bin_check([img_x[c['pixel_y'],c['pixel_x']], img_y[c['pixel_y'],c['pixel_x']], img_z[c['pixel_y'],c['pixel_x']]], bin)
                    if not inside:
                        reject_map[c['pixel_y'],c['pixel_x']] = 5 # outside bin
                        continue
                elif c['approach_strategy']=='left':
                    inside = LeftRight_arm.inside_bin_check([img_x[c['pixel_y'],c['pixel_x']], img_y[c['pixel_y'],c['pixel_x']]+30, img_z[c['pixel_y'],c['pixel_x']]], bin)
                    if not inside:
                        reject_map[c['pixel_y'],c['pixel_x']] = 5 # outside bin
                        continue
                elif c['approach_strategy']=='right':
                    inside = LeftRight_arm.inside_bin_check([img_x[c['pixel_y'],c['pixel_x']], img_y[c['pixel_y'],c['pixel_x']]-30, img_z[c['pixel_y'],c['pixel_x']]], bin)
                    if not inside:
                        reject_map[c['pixel_y'],c['pixel_x']] = 5 # outside bin
                        continue
                
                #print len(ps), len(ps2)
                #print ln
                #print nrm
                
                c['normal'] = nrm
                c['target_pos'] = np.asarray([img_x[c['pixel_y'],c['pixel_x']], img_y[c['pixel_y'],c['pixel_x']], img_z[c['pixel_y'],c['pixel_x']]])
                ret[i].append(c)
        return ret
