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

import numpy as np
import math as ma
# the robot origin is 120 cm from floor. the shelf origin is 83 cm from floor
class conv_bin_coord:
    def __init__(self):
        self.shelf_base_pos = {}
        self.base_origin = np.asarray([1250,5,-372,0,0,0]) # global coordinate of bin J origin (before adjustment) #-352
        self.base_p1 = np.asarray([1250,40,1037,0,0,0]) #previous:1057 
        dy = [-32, -283, -588]
        dz = [0, 270, 498, 728]
        self.shelf_base_pos["bin_A_o"] = np.asarray([0, dy[0], dz[3], 0, 0, 0])
        self.shelf_base_pos["bin_B_o"] = np.asarray([0, dy[1], dz[3], 0, 0, 0])
        self.shelf_base_pos["bin_C_o"] = np.asarray([0, dy[2], dz[3], 0, 0, 0])
        self.shelf_base_pos["bin_D_o"] = np.asarray([0, dy[0], dz[2], 0, 0, 0])
        self.shelf_base_pos["bin_E_o"] = np.asarray([0, dy[1], dz[2], 0, 0, 0])
        self.shelf_base_pos["bin_F_o"] = np.asarray([0, dy[2], dz[2], 0, 0, 0])
        self.shelf_base_pos["bin_G_o"] = np.asarray([0, dy[0], dz[1], 0, 0, 0])
        self.shelf_base_pos["bin_H_o"] = np.asarray([0, dy[1], dz[1], 0, 0, 0])
        self.shelf_base_pos["bin_I_o"] = np.asarray([0, dy[2], dz[1], 0, 0, 0])
        self.shelf_base_pos["bin_J_o"] = np.asarray([0, dy[0], dz[0], 0, 0, 0])
        self.shelf_base_pos["bin_K_o"] = np.asarray([0, dy[1], dz[0], 0, 0, 0])
        self.shelf_base_pos["bin_L_o"] = np.asarray([0, dy[2], dz[0], 0, 0, 0])
    
    # shelf coordinate <-> bin coordinate
    def conv_to_shelf_base(self, point, binname):
        print "shelf",point + self.shelf_base_pos[binname + '_o']
        return point + self.shelf_base_pos[binname + '_o']
    def conv_from_shelf_base(self, point, binname):
        return point - self.shelf_base_pos[binname + '_o']

    
    def conv_to_global_real(self, point, p_1, p_2):
        print p_1, p_2
        deg = ma.atan2((p_2[1]-p_1[1]),(p_2[0]-p_1[0])) + ma.pi/2
        print 'deg=',ma.degrees(deg)
        p = point.copy()
        print "kaiten", p[0] * ma.cos(deg) - p[1] * ma.sin(deg),p[0] * ma.sin(deg) + p[1] * ma.cos(deg) 
        point[0] = p[0] * ma.cos(deg) - p[1] * ma.sin(deg) + p_1[0]
        point[1] = p[0] * ma.sin(deg) + p[1] * ma.cos(deg) + p_1[1]
        point[2] = p[2] + self.base_origin[2]
        point[5] = p[5] + ma.degrees(deg) 
        return point

    def conv_from_global_real(self, point, p_1, p_2):
        deg = - ma.atan2((p_2[1]-p_1[1]),(p_2[0]-p_1[0])) - ma.pi/2
        print 'deg=',ma.degrees(deg)
        p = point.copy()
        p[0] = p[0] - p_1[0]
        p[1] = p[1] - p_1[1]
        point[0] = p[0] * ma.cos(deg) - p[1] * ma.sin(deg)
        point[1] = p[0] * ma.sin(deg) + p[1] * ma.cos(deg)
        point[2] = p[2] - self.base_origin[2]
        point[5] = p[5] - ma.degrees(deg)
        return point
        
    def adjustglobal(self, point, p_1, p_2):
        deg = ma.atan2((p_2[1]-p_1[1]),(p_2[0]-p_1[0])) + ma.pi/2
        print 'deg=', ma.degrees(deg)
        p = point.copy()
        p[0] = p[0] - self.base_p1[0]
        p[1] = p[1] - self.base_p1[1]
        point[0] = p[0] * ma.cos(deg) - p[1] * ma.sin(deg) + p_1[0]
        point[1] = p[0] * ma.sin(deg) + p[1] * ma.cos(deg) + p_1[1]
        point[5] = p[5] + ma.degrees(deg)
        return point 

    def conv_from_bin_coord_to_global_coord(self,point,bin, p_1, p_2):
        print 'Bin Coordinate from',point, 'of', bin
        result = self.conv_to_global_real(self.conv_to_shelf_base(point,bin), p_1, p_2)
        print 'to Global Coordinate', result
        return result
    def conv_to_bin_coord_from_global_coord(self,point,bin,p_1,p_2):
        print 'from Global Coordinate',point
        result =  self.conv_from_shelf_base(self.conv_from_global_real(point,p_1,p_2), bin)
        print 'to Bin Coordinate', result
        return result
if __name__ == "__main__":
    a = conv_bin_coord()
    print 'global -> bin'
    print a.conv_to_bin_coord_from_global_coord(np.asarray([0,0,0,0,0,0]),'bin_A',[1,2,3],[1,2,3])
    print 'bin -> global'
    print a.conv_from_bin_coord_to_global_coord(np.asarray([0,0,0,0,0,0]),'bin_A',[1,2,3],[1,2,3])
