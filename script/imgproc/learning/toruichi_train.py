#!/usr/bin/env python
# coding: utf-8

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

import sys
sys.path.append("/home/user/anaconda2/lib/python2.7/site-packages/")
import math
import os
import pickle

import chainer
from chainer import cuda
from chainer import optimizers
from chainer import serializers
from chainer import Variable

import chainer.functions as F
import chainer.links as L

import cv2
import random
import shutil
import numpy as np
import sklearn.utils
from PIL import Image
from scipy.ndimage.interpolation import zoom
xp = cuda.cupy

# Current strategy:
# * fully-convolutional/deconvolutional networks
# * use a pre-trained model with CG dataset
# * use both bin0513 and stow0527 datasets
# * apply data augmentation with hsv, gaussian, white noise, and rotate filters

# Global settings
out_image_dir = './result/toruichi_out_images0_2'
out_model_dir = './result/toruichi_out_models0_2'
#dataset_dir = "/home/user/apc/image_processing/dataset/dataset"
batchsize = 25
log_itv = 5000
gpu_id = 3


def _get_image_and_label():
    print 'Loading dataset...'
    # Bin dataset, (600, 3, 280, 440)
    xs1 = np.load('/data/user/apc/annotation_0513/images_0513.npz')['arr_0']
    ds1 = np.load('/data/user/apc/annotation_0513/depths_0513.npz')['arr_0']
    ys1 = np.load('/data/user/apc/annotation_0513/labels_0513.npz')['arr_0']

    # Stow dataset, (100, 3, 480, 640)
    xs2 = np.load('/data/user/apc/stow_0527/images_0527.npz')['arr_0']
    ds2 = np.load('/data/user/apc/stow_0527/depths_0527.npz')['arr_0']
    ys2 = np.load('/data/user/apc/stow_0527/labels_0527.npz')['arr_0']

    # Bin&Stow++ dataset, (721, 3, 480, 640)
    xs3 = np.load('/data/user/apc/annotation_0603/images_0603.npz')['arr_0']
    ds3 = np.load('/data/user/apc/annotation_0603/depths_0603.npz')['arr_0']
    ys3 = np.load('/data/user/apc/annotation_0603/labels_0603.npz')['arr_0']
    ts3 = np.load('/data/user/apc/annotation_0603/toruichi_0603.npz')['arr_0']

    # preprocessing
    ds1, ds2, ds3 = ds1[:, 0, :, :], ds2[:, 0, :, :], ds3[:, 0, :, :]
    ts3 = ts3[:,0,:,:]
    ys1 = ys1[:, :280, :]  # discard the 'trash'

    # resize/crop previous data due to the difference of aspect ratio
    # bilinear interpolation is extremely slow!
    ratio1 = 240.0 / 280.0
    xs1 = zoom(xs1, (1.0, 1.0, ratio1, ratio1), order=0)
    ds1 = zoom(ds1, (1.0, ratio1, ratio1), order=0)
    ys1 = zoom(ys1, (1.0, ratio1, ratio1), order=0)
    ratio2 = 320.0 / 640.0
    xs2 = zoom(xs2, (1.0, 1.0, ratio2, ratio2), order=0)
    ds2 = zoom(ds2, (1.0, ratio2, ratio2), order=0)
    ys2 = zoom(ys2, (1.0, ratio2, ratio2), order=0)
    ratio3 = 320.0 / 640.0
    xs3 = zoom(xs3, (1.0, 1.0, ratio3, ratio3), order=0)
    ds3 = zoom(ds3, (1.0, ratio3, ratio3), order=0)
    ys3 = zoom(ys3, (1.0, ratio3, ratio3), order=0)
    ts3 = zoom(ts3, (1.0, ratio3, ratio3), order=0)

    u0 = (xs1.shape[3] - xs2.shape[3]) // 2
    u1 = u0 + xs2.shape[3]
    xs1 = xs1[:, :, :, u0:u1]
    ds1 = ds1[:, :, u0:u1]
    ys1 = ys1[:, :, u0:u1]

    N1, N2, N3 = xs1.shape[0], xs2.shape[0], xs3.shape[0]
    xs = np.empty((N1 + N2 + N3, 4) + xs2.shape[2:], dtype=xs2.dtype)
    ys = np.empty((N1 + N2 + N3,) + ys2.shape[1:], dtype=ys2.dtype)
    ts = np.empty((N1 + N2 + N3,) + ys2.shape[1:], dtype=ys2.dtype)

    xs[:N1, :3, :, :] = xs1
    xs[:N1, 3, :, :] = ds1
    xs[N1:(N1 + N2), :3, :, :] = xs2
    xs[N1:(N1 + N2), 3, :, :] = ds2
    xs[(N1 + N2):, :3, :, :] = xs3
    xs[(N1 + N2):, 3, :, :] = ds3
    ys[:N1] = ys1
    ys[N1:(N1 + N2)] = ys2
    ys[(N1 + N2):] = ys3
    ts[(N1 + N2):] = ts3

    print 'Done.'
    return xs, ys, ts

def get_train_image_and_label():
    return _get_image_and_label()

def get_test_image_and_label():
    return _get_image_and_label()

def crop_and_mirror(img, cont, toru, test=True):
    h,w = img.shape[1], img.shape[2]
    if h>224:
        oh = np.random.randint(h-224)
    else:
        oh = 0
    if w>224:
        ow = np.random.randint(w-224)
    else:
        ow = 0
    mirror_flag = np.random.randint(2)

    img, cont, toru = img[:, oh:(oh+224), ow:(ow+224)], cont[oh:(oh+224), ow:(ow+224)], toru[oh:(oh+224), ow:(ow+224)]
    if mirror_flag == 1:
        img, cont, toru = img[:,:,::-1], cont[:,::-1], toru[:,::-1]

    if test:
        return img, cont, toru
    else:
        return add_noise(img, cont, toru)

def add_noise(image, cont, toru):
    # Apply some filters to the given image.
    # The current filters we have implemented are:
    def _hsv_filter(img):
        assert(img.shape[2] == 3)
        hsv_int32 = cv2.cvtColor(img, cv2.COLOR_RGB2HSV_FULL).astype(np.int32)
        hsv_uint8 = np.empty_like(hsv_int32, dtype=np.uint8)
        H, S, V = int(255 * 0.05), int(255 * 0.1), int(255 * 0.1)
        h = random.randint(-H, H)
        s = random.randint(-S, S)
        v = random.randint(-V, V)
        hsv_int32[:, :, 0] += h
        hsv_int32[:, :, 1] += s
        hsv_int32[:, :, 2] += v
        hsv_uint8[:, :, 0] = np.remainder(hsv_int32[:, :, 0], 255)
        hsv_uint8[:, :, 1] = np.clip(hsv_int32[:, :, 1], 0, 255)
        hsv_uint8[:, :, 2] = np.clip(hsv_int32[:, :, 2], 0, 255)
        return cv2.cvtColor(hsv_uint8, cv2.COLOR_HSV2RGB_FULL)

    def hsv_filter(img):
        if img.shape[2] == 3:
            return _hsv_filter(img)
        elif img.shape[2] == 4:
            rgb_img, depth_img = img[:, :, :3], img[:, :, 3]
            dst = np.zeros_like(img, dtype=np.uint8)
            rgb_img = _hsv_filter(rgb_img)
            dst[:, :, :3] = rgb_img
            dst[:, :, 3] = depth_img
            return dst
        else:
            raise TypeError

    def gaussian_blur(img):
        ksize = random.randrange(1, 7 + 1, 2)
        dst_image = cv2.GaussianBlur(src=img, ksize=(ksize, ksize), sigmaX=0)
        return dst_image

    def white_noise(img):
        sigma = random.uniform(0.0, 0.2)
        dst_image = np.clip(
            img + (255.0 * sigma) * np.random.randn(*img.shape), 0, 255)
        dst_image = dst_image.astype(np.uint8)
        return dst_image

    def rotate(img, cont, toru):
        # rotate img and cont depending on the following prob.
        flag = np.random.randint(10)
        if flag < 6:
            return img, cont, toru
        elif flag < 8:
            return np.rot90(img, k=1), np.rot90(cont, k=1), np.rot90(toru, k=1)
        else:
            return np.rot90(img, k=3), np.rot90(cont, k=3), np.rot90(toru, k=3)

    img = ((image + 1) * 128).astype(np.uint8).transpose([1, 2, 0])

    img = white_noise(gaussian_blur(hsv_filter(img)))
    img, cont, toru = rotate(img, cont, toru)

    img = ((img - 128.0) / 128.0).transpose([2, 0, 1]).astype(np.float32)
    return img, cont, toru


class Encoder(chainer.Chain):

    def __init__(self):
        layers = {}
        layers['conv1_1'] = L.Convolution2D(4, 64, 3, 1, 1)
#        layers['conv1_2'] = L.Convolution2D(64, 64, 3, 1, 1)
        layers['conv1_3'] = L.Convolution2D(64, 64, 4, 2, 1)
        layers['conv2_1'] = L.Convolution2D(64, 64, 3, 1, 1)
#        layers['conv2_2'] = L.Convolution2D(64, 64, 3, 1, 1)
        layers['conv2_3'] = L.Convolution2D(64, 64, 4, 2, 1)
        layers['conv3_1'] = L.Convolution2D(64, 64, 3, 1, 1)
#        layers['conv3_2'] = L.Convolution2D(64, 64, 3, 1, 1)
        layers['conv3_3'] = L.Convolution2D(64, 64, 4, 2, 1)
        layers['conv4_1'] = L.Convolution2D(64, 64, 3, 1, 1)
#        layers['conv4_2'] = L.Convolution2D(64, 64, 3, 1, 1)
        layers['conv4_3'] = L.Convolution2D(64, 64, 4, 2, 1)
        layers['conv5_1'] = L.Convolution2D(64, 128, 3, 1, 1)
#        layers['conv5_2'] = L.Convolution2D(128, 256, 3, 1, 1)
        layers['conv5_3'] = L.Convolution2D(128, 256, 3, 1, 1)
        layers['bn1_1'] = L.BatchNormalization(64)
        layers['bn1_2'] = L.BatchNormalization(64)
        layers['bn1_3'] = L.BatchNormalization(64)
        layers['bn2_1'] = L.BatchNormalization(64)
        layers['bn2_2'] = L.BatchNormalization(64)
        layers['bn2_3'] = L.BatchNormalization(64)
        layers['bn3_1'] = L.BatchNormalization(64)
        layers['bn3_2'] = L.BatchNormalization(64)
        layers['bn3_3'] = L.BatchNormalization(64)
        layers['bn4_1'] = L.BatchNormalization(64)
        layers['bn4_2'] = L.BatchNormalization(64)
        layers['bn4_3'] = L.BatchNormalization(64)
        layers['bn5_1'] = L.BatchNormalization(128)
        layers['bn5_2'] = L.BatchNormalization(256)
        super(Encoder, self).__init__(**layers)

    def __call__(self, x, test):
        h = F.relu(self.bn1_1(self.conv1_1(x), test=test))
#        h = F.relu(self.bn1_2(self.conv1_2(h), test=test))
        h = F.relu(self.bn1_3(self.conv1_3(h), test=test))
        h = F.relu(self.bn2_1(self.conv2_1(h), test=test))
#        h = F.relu(self.bn2_2(self.conv2_2(h), test=test))
        h = F.relu(self.bn2_3(self.conv2_3(h), test=test))
        h = F.relu(self.bn3_1(self.conv3_1(h), test=test))
#        h = F.relu(self.bn3_2(self.conv3_2(h), test=test))
        h = F.relu(self.bn3_3(self.conv3_3(h), test=test))
        h = F.relu(self.bn4_1(self.conv4_1(h), test=test))
#        h = F.relu(self.bn4_2(self.conv4_2(h), test=test))
        h = F.relu(self.bn4_3(self.conv4_3(h), test=test))
        h = F.relu(self.bn5_1(self.conv5_1(h), test=test))
#        h = F.relu(self.bn5_2(self.conv5_2(h), test=test))
        y = self.conv5_3(h)
        return y


class Decoder(chainer.Chain):

    n_outchannels = 40

    def __init__(self):
        layers = {}
        layers['dc0_0'] = L.Deconvolution2D(256, 256, 1, 1, 0, wscale=0.02*math.sqrt(1*1*512))
        layers['dc0_1'] = L.Deconvolution2D(256, 128, 4, 2, 1, wscale=0.02*math.sqrt(4*4*256))
        layers['dc1_0'] = L.Deconvolution2D(128, 128, 3, 1, 1, wscale=0.02*math.sqrt(3*3*128))
#        layers['dc1_1'] = L.Deconvolution2D(128, 128, 3, 1, 1, wscale=0.02*math.sqrt(3*3*128))
        layers['dc1_2'] = L.Deconvolution2D(128, 64, 4, 2, 1, wscale=0.02*math.sqrt(4*4*128))
        layers['dc2_0'] = L.Deconvolution2D(64, 64, 3, 1, 1, wscale=0.02*math.sqrt(3*3*64))
#        layers['dc2_1'] = L.Deconvolution2D(64, 64, 3, 1, 1, wscale=0.02*math.sqrt(3*3*64))
        layers['dc2_2'] = L.Deconvolution2D(64, 64, 4, 2, 1, wscale=0.02*math.sqrt(4*4*64))
        layers['dc3_0'] = L.Deconvolution2D(64, 64, 3, 1, 1, wscale=0.02*math.sqrt(3*3*64))
#        layers['dc3_1'] = L.Deconvolution2D(64, 64, 3, 1, 1, wscale=0.02*math.sqrt(3*3*64))
        layers['dc3_2'] = L.Deconvolution2D(64, 64, 4, 2, 1, wscale=0.02*math.sqrt(4*4*64))
        layers['dc4_0'] = L.Deconvolution2D(64, 64, 3, 1, 1, wscale=0.02*math.sqrt(3*3*64))
        layers['dc4_1'] = L.Deconvolution2D(64, self.n_outchannels, 3, 1, 1, wscale=0.02*math.sqrt(3*3*64))
        layers['bn0_0'] = L.BatchNormalization(256)
        layers['bn0_1'] = L.BatchNormalization(128)
        layers['bn1_0'] = L.BatchNormalization(128)
        layers['bn1_1'] = L.BatchNormalization(128)
        layers['bn1_2'] = L.BatchNormalization(64)
        layers['bn2_0'] = L.BatchNormalization(64)
        layers['bn2_1'] = L.BatchNormalization(64)
        layers['bn2_2'] = L.BatchNormalization(64)
        layers['bn3_0'] = L.BatchNormalization(64)
        layers['bn3_1'] = L.BatchNormalization(64)
        layers['bn3_2'] = L.BatchNormalization(64)
        layers['bn4_0'] = L.BatchNormalization(64)
        super(Decoder, self).__init__(**layers)

    def __call__(self, x, test):
        h = F.leaky_relu(self.bn0_0(self.dc0_0(x), test=test))
        h = F.leaky_relu(self.bn0_1(self.dc0_1(h), test=test))
        h = F.leaky_relu(self.bn1_0(self.dc1_0(h), test=test))
#        h = F.leaky_relu(self.bn1_1(self.dc1_1(h), test=test))
        h = F.leaky_relu(self.bn1_2(self.dc1_2(h), test=test))
        h = F.leaky_relu(self.bn2_0(self.dc2_0(h), test=test))
#        h = F.leaky_relu(self.bn2_1(self.dc2_1(h), test=test))
        h = F.leaky_relu(self.bn2_2(self.dc2_2(h), test=test))
        h = F.leaky_relu(self.bn3_0(self.dc3_0(h), test=test))
#        h = F.leaky_relu(self.bn3_1(self.dc3_1(h), test=test))
        h_ = F.leaky_relu(self.bn3_2(self.dc3_2(h), test=test))
        h = F.leaky_relu(self.bn4_0(self.dc4_0(h_), test=test))
        h = (self.dc4_1(h))
        return h

class TDecoder(Decoder):
    n_outchannels = 2

def make_batch(imgs_train, imgs_test, labels_train, labels_test, toruichi_train, toruichi_test, test, toruichi):
    x = np.zeros((batchsize, 4, 224, 224)).astype(np.float32)
    t = np.zeros((batchsize, 224, 224)).astype(np.int32)
    t2 = np.zeros((batchsize, 224, 224)).astype(np.int32)

    for i in range(batchsize):
        m = imgs_train.shape[0] - 721 if toruichi else 0
        if not test:
            r = np.random.randint(m,imgs_train.shape[0])
            img, cont, toru = imgs_train[r], labels_train[r], toruichi_train[r]
            v = False
        else:
            r = np.random.randint(m,imgs_test.shape[0])
            img, cont, toru = imgs_test[r], labels_test[r], toruichi_test[r]
            v = True

        img_c, cont_c, toru_c = crop_and_mirror(img, cont, toru, test=test)
        x[i,:] = img_c
        t[i,:] = cont_c
        t2[i,:] = toru_c
    return Variable(xp.asarray(x), volatile=v), Variable(xp.asarray(t), volatile=v), Variable(xp.asarray(t2), volatile=v)


def train(enc, dec, tdec, initial_epoch=0):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    o_enc = optimizers.Adam(alpha=0.0001*(0.98**initial_epoch))
    o_dec = optimizers.Adam(alpha=0.0001*(0.98**initial_epoch))
    o_tdec = optimizers.Adam(alpha=0.001*(0.98**initial_epoch))
    o_enc.setup(enc)
    o_dec.setup(dec)
    o_tdec.setup(tdec)
    decay_enc = 0.0001
    decay_dec = 0.0001
    decay_tdec = 0.0001
    o_enc.add_hook(chainer.optimizer.WeightDecay(decay_enc), 'hook_enc')
    o_dec.add_hook(chainer.optimizer.WeightDecay(decay_dec), 'hook_dec')
    o_tdec.add_hook(chainer.optimizer.WeightDecay(decay_tdec), 'hook_tdec')

    imgs_train, labels_train, toruichi_train  = get_train_image_and_label()
    # XXX: validation dataset is the same as the training one.
    imgs_test, labels_test, toruichi_test = imgs_train, labels_train, toruichi_train

    loss_weight = None
    for epoch in range(initial_epoch, 10000):
        if loss_weight is None:
            loss_weight = xp.asarray(
                sklearn.utils.compute_class_weight(
                    'balanced', np.unique(labels_train.flat), labels_train.flat),
                dtype=np.float32)

        o_enc.alpha = o_enc.alpha*0.98
        o_dec.alpha = o_dec.alpha*0.98
        print imgs_train.shape, labels_train.shape
        sum_loss = 0
        for it in range(0, 10000, batchsize):
            print epoch, it
            if np.random.randint(2)==0:
                toruichi = True
            else:
                toruichi = False
            x, t, t2 = make_batch(
                imgs_train, imgs_test, labels_train, labels_test, toruichi_train, toruichi_test, test=False, toruichi=toruichi)
            z = enc(x, test=False)
            y = dec(z, test=False)
            yd = Variable(y.data)
            loss = F.softmax_cross_entropy(yd, t)
            if toruichi:
                loss += F.softmax_cross_entropy(tdec(z, test=False), t2)

            o_enc.zero_grads()
            o_dec.zero_grads()
            o_tdec.zero_grads()
            loss.backward()
            y.grad = loss_weight.reshape(1, loss_weight.size, 1, 1) * yd.grad
            y.backward()
            o_enc.update()
            o_dec.update()
            o_tdec.update()
            loss.unchain_backward()
            y.unchain_backward()

            sum_loss += loss.data.get()

            if (it+batchsize)%log_itv == 0:
                sum_loss_test = 0
                for i_test in range(1):
                    x, t, t2 = make_batch(
                        imgs_train, imgs_test, labels_train, labels_test, toruichi_train, toruichi_test, test=True, toruichi=False)
                    z = enc(x, test=True)
                    y = dec(z, test=True)
                    toru = tdec(z, test=True)
                    loss = F.softmax_cross_entropy(y, t)
                    loss.unchain_backward()
                    toru.unchain_backward()
                    sum_loss_test += loss.data.get()
                print epoch, it, sum_loss/log_itv*batchsize, sum_loss_test
                plt.rcParams['figure.figsize'] = (20.0,20.0)
                plt.clf()


                for i in range(16):
                    plt.subplot(8, 8, i*4+1)
                    plt.imshow(((x.data.get()[i,:3]+1)/2).transpose(1,2,0))
                    plt.axis('off')

                    plt.subplot(8, 8, i*4+2)
                    plt.imshow(t.data.get()[i,:], cmap=plt.cm.Paired, vmin=0, vmax=40)
                    plt.axis('off')

                    plt.subplot(8, 8, i*4+3)
                    plt.imshow((np.argmax(y.data.get(), axis=1)[i,:]), cmap=plt.cm.Paired, vmin=0, vmax=40)
                    plt.axis('off')

                    plt.subplot(8, 8, i*4+4)
                    plt.imshow(toru.data.get()[i,0,:], cmap=plt.cm.gray, vmin=0, vmax=1)
                    plt.axis('off')

                plt.savefig('%s/vis_%d_%d.png'%(out_image_dir, epoch,it))

                f_log = open('%s/loss_log.txt'%out_model_dir,'a')
                f_log.write("%d %d %f %f\n"%(epoch, it, sum_loss/log_itv*batchsize, sum_loss_test))
                f_log.close()

                sum_loss = 0

        serializers.save_npz("%s/model_enc_%d.npz"%(out_model_dir, epoch),enc)
        serializers.save_npz("%s/model_dec_%d.npz"%(out_model_dir, epoch),dec)
        serializers.save_npz("%s/model_tdec_%d.npz"%(out_model_dir, epoch),tdec)


def mkdir(path):
    try:
        os.mkdir(path)
        return True
    except:
        return False


if __name__ == '__main__':
    cuda.get_device(gpu_id).use()

    enc = Encoder()
    serializers.load_npz('/home/user/work/apc2016/script/imgproc/learning/result/segmentation_out_models10_8/model_enc_95.npz', enc)

    dec = Decoder()
    serializers.load_npz('/home/user/work/apc2016/script/imgproc/learning/result/segmentation_out_models10_8/model_dec_95.npz', dec)

    #serializers.load_hdf5("./segmentation_out_models6/model_dec_%d.h5"%(121),dec)
    #serializers.load_hdf5('./out_models/model_dec_17.h5',dec)

    tdec = TDecoder()

    dec.to_gpu()
    enc.to_gpu()
    tdec.to_gpu()

    mkdir(out_image_dir)
    mkdir(out_model_dir)
    shutil.copy('toruichi_train.py', out_model_dir)

    train(enc, dec, tdec, initial_epoch=0)
