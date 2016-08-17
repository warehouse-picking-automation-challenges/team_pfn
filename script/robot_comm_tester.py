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

import matplotlib,  struct, time
matplotlib.use("Agg")
import numpy as np
import matplotlib.backends.backend_agg as agg

import pylab
import pickle
import time

import pygame
from pygame.locals import *


class Visualizer:
    def __init__(self):
        pygame.init()
        self.width = 400
        self.height = 400
        self.window = pygame.display.set_mode((self.width, self.height), DOUBLEBUF)
        self.screen = pygame.display.get_surface()

        self.fig = pylab.figure(figsize=[4, 4], # Inches
                           dpi=100,        # 100 dots per inch, so the resulting buffer is 400x400 pixels
                           )

        canvas = agg.FigureCanvasAgg(self.fig)
        self.size = canvas.get_width_height()


visualizer = Visualizer()


import socket, re, time

import socket
import datetime
import math
import time

def sendmsg(x,y,z,w,p,r):
    msg = "%.3f %.3f %.3f %.3f %.3f %.3f"%(x,y,z,w,p,r)
    soc.sendall(msg+"\n")

    now = datetime.datetime.now()
    print "send, ", msg, step, now.strftime("%M %S"), now.microsecond

    data = soc.recv(1024)
    print "recv, ", data

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind(("", 59002))
s.listen(1)
soc, addr = s.accept()
print "Connected by", str(addr)

NT = 100

step = 0
x, y, z = 790,0,700 #500, 0, 40
w0,p0,r0 = 180,0,180 #0, 90, 0

sendmsg(x,y,z,w0,p0,r0)

while True:
    pygame.event.pump()
    pressed_keys = pygame.key.get_pressed()
    if pressed_keys[K_r]:
        z+=10
    if pressed_keys[K_f]:
        z-=10
    if pressed_keys[K_w]:
        x+=10
    if pressed_keys[K_s]:
        x-=10
    if pressed_keys[K_a]:
        y-=10
    if pressed_keys[K_d]:
        y+=10
    if pressed_keys[K_t]:
        w0+=10
    if pressed_keys[K_g]:
        w0-=10
    if pressed_keys[K_y]:
        p0+=10
    if pressed_keys[K_h]:
        p0-=10
    if pressed_keys[K_u]:
        r0+=10
    if pressed_keys[K_j]:
        r0-=10

    x = 500 if x<500 else (1000 if x>1000 else x)
    y = -500 if y<-500 else (500 if y>500 else y)
    z = -200 if z<-200 else (700 if z>700 else z)
    sendmsg(x,y,z,w0,p0,r0)


    #visualizer.plot_array(sensor_value)
    step += 1
    time.sleep(0.05)
