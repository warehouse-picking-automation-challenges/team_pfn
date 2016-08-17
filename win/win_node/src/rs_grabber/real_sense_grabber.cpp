// Copyright 2016 Preferred Networks, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// This file incorporates code covered by the following terms:

/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2015-, Open Perception, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <boost/lexical_cast.hpp>

#include <math.h>

#include <pxcimage.h>
#include <pxccapture.h>
#include <pxcprojection.h>
#include <pxcsensemanager.h>

#include <pcl/common/io.h>
#include <pcl/common/time.h>

#include "real_sense_grabber.h"
#include "real_sense/real_sense_device_manager.h"
#include "buffers.h"
#include "pcl/io/io_exception.h"

using namespace pcl::io;
using namespace pcl::io::real_sense;

/* Helper function to convert a PXCPoint3DF32 point into a PCL point.
 * Takes care of unit conversion (PXC point coordinates are in millimeters)
 * and invalid points. */
template <typename T> inline void
convertPoint (const PXCPoint3DF32& src, T& tgt)
{
  static const float nan = std::numeric_limits<float>::quiet_NaN ();
  if (src.z == 0)
  {
    tgt.x = tgt.y = tgt.z = nan;
  }
  else
  {
    tgt.x = -src.x / 1000.0;
    tgt.y = src.y / 1000.0;
    tgt.z = src.z / 1000.0;
  }
}

pcl::RealSenseGrabber::Mode::Mode ()
: fps (0), depth_width (0), depth_height (0), color_width (0), color_height (0)
{
}

pcl::RealSenseGrabber::Mode::Mode (unsigned int f)
: fps (f), depth_width (0), depth_height (0), color_width (0), color_height (0)
{
}

pcl::RealSenseGrabber::Mode::Mode (unsigned int dw, unsigned int dh)
: fps (0), depth_width (dw), depth_height (dh), color_width (0), color_height (0)
{
}

pcl::RealSenseGrabber::Mode::Mode (unsigned int f, unsigned int dw, unsigned int dh)
: fps (f), depth_width (dw), depth_height (dh), color_width (0), color_height (0)
{
}

pcl::RealSenseGrabber::Mode::Mode (unsigned int dw, unsigned int dh, unsigned int cw, unsigned int ch)
: fps (0), depth_width (dw), depth_height (dh), color_width (cw), color_height (ch)
{
}

pcl::RealSenseGrabber::Mode::Mode (unsigned int f, unsigned int dw, unsigned int dh, unsigned int cw, unsigned int ch)
: fps (f), depth_width (dw), depth_height (dh), color_width (cw), color_height (ch)
{
}

bool
operator== (const pcl::RealSenseGrabber::Mode& m1, const pcl::RealSenseGrabber::Mode& m2)
{
  return (m1.fps == m2.fps &&
          m1.depth_width == m2.depth_width &&
          m1.depth_height == m2.depth_height &&
          m1.color_width == m2.color_width &&
          m1.color_height == m2.color_height);
}

pcl::RealSenseGrabber::RealSenseGrabber (const std::string& device_id, const Mode& mode, bool strict)
: Grabber ()
, is_running_ (false)
, confidence_threshold_ (6)
, temporal_filtering_type_ (RealSense_None)
, temporal_filtering_window_size_ (1)
, mode_requested_ (mode)
, strict_ (strict)
{
  if (device_id == "")
    device_ = RealSenseDeviceManager::getInstance ()->captureDevice ();
  else if (device_id[0] == '#')
    device_ = RealSenseDeviceManager::getInstance ()->captureDevice (boost::lexical_cast<int> (device_id.substr (1)) - 1);
  else
    device_ = RealSenseDeviceManager::getInstance ()->captureDevice (device_id);

  point_cloud_signal_ = createSignal<sig_cb_real_sense_point_cloud> ();
  point_cloud_rgba_signal_ = createSignal<sig_cb_real_sense_point_cloud_rgba> ();
  xyzrgba_sepa_signal_ = createSignal<sig_cb_real_sense_xyzrgba_sepa>();
}

pcl::RealSenseGrabber::~RealSenseGrabber () throw ()
{
  stop ();

  disconnect_all_slots<sig_cb_real_sense_point_cloud> ();
  disconnect_all_slots<sig_cb_real_sense_point_cloud_rgba> ();
}

void
pcl::RealSenseGrabber::start ()
{

	device_->getPXCDevice().SetColorAutoExposure(false);
	//device_->getPXCDevice().SetColorAutoExposure(true);
	device_->getPXCDevice().SetColorAutoWhiteBalance(false);
	//device_->getPXCDevice().SetColorAutoWhiteBalance(true);

	PXCCapture::Device::PropertyInfo info = device_->getPXCDevice().QueryColorExposureInfo();

	printf("color exposure info (%f,%f), %f, val = %f\n", info.range.min, info.range.max, info.step, info.defaultValue);

	int val = -4;
	printf("set color exposure :%d  result:%d\n", val, device_->getPXCDevice().SetColorExposure(val));


  if (!is_running_)
  {
    need_xyz_ = num_slots<sig_cb_real_sense_point_cloud> () > 0;
//    need_xyzrgba_ = num_slots<sig_cb_real_sense_point_cloud_rgba> () > 0;
//	need_xyzrgba_sepa = num_slots<sig_cb_real_sense_xyzrgba_sepa>() > 0;

    if (need_xyz_ || need_xyzrgba_ || need_xyzrgba_sepa)
    {
      selectMode ();
      PXCCapture::Device::StreamProfileSet profile;
      memset (&profile, 0, sizeof (profile));
      profile.depth.frameRate.max = mode_selected_.fps;
      profile.depth.frameRate.min = mode_selected_.fps;
      profile.depth.imageInfo.width = mode_selected_.depth_width;
      profile.depth.imageInfo.height = mode_selected_.depth_height;
      profile.depth.imageInfo.format = PXCImage::PIXEL_FORMAT_DEPTH;		//16bit mm
      profile.depth.options = PXCCapture::Device::STREAM_OPTION_ANY;

      if (need_xyzrgba_ || need_xyzrgba_sepa)
      {
        profile.color.frameRate.max = mode_selected_.fps;
        profile.color.frameRate.min = mode_selected_.fps;
        profile.color.imageInfo.width = mode_selected_.color_width;
        profile.color.imageInfo.height = mode_selected_.color_height;
        profile.color.imageInfo.format = PXCImage::PIXEL_FORMAT_RGB32;
        profile.color.options = PXCCapture::Device::STREAM_OPTION_ANY;
      }

	  pxcStatus st = device_->getPXCDevice ().SetStreamProfileSet (&profile);
	  //printf("st %d\n", st);
      if (!device_->getPXCDevice ().IsStreamProfileSetValid (&profile))
        THROW_IO_EXCEPTION ("Invalid stream profile for PXC device");
      frequency_.reset ();
      is_running_ = true;
      thread_ = boost::thread (&RealSenseGrabber::run, this);
    }
	//const int WIDTH = mode_selected_.depth_width;
	//const int HEIGHT = mode_selected_.depth_height;
	//printf("w h %d %d\n", WIDTH, HEIGHT);

  }
}

void
pcl::RealSenseGrabber::stop ()
{
  if (is_running_)
  {
    is_running_ = false;
    thread_.join ();
  }
}

bool
pcl::RealSenseGrabber::isRunning () const
{
  return (is_running_);
}

float
pcl::RealSenseGrabber::getFramesPerSecond () const
{
  boost::mutex::scoped_lock lock (fps_mutex_);
  return (frequency_.getFrequency ());
}

void
pcl::RealSenseGrabber::setConfidenceThreshold (unsigned int threshold)
{
  if (threshold > 15)
  {
    PCL_WARN ("[pcl::RealSenseGrabber::setConfidenceThreshold] Attempted to set threshold outside valid range (0-15)");
  }
  else
  {
    confidence_threshold_ = threshold;
    device_->getPXCDevice ().SetDepthConfidenceThreshold (confidence_threshold_);
  }
}

void
pcl::RealSenseGrabber::enableTemporalFiltering (TemporalFilteringType type, size_t window_size)
{
  if (temporal_filtering_type_ != type ||
     (type != RealSense_None && temporal_filtering_window_size_ != window_size))
  {
    temporal_filtering_type_ = type;
    temporal_filtering_window_size_ = window_size;
    if (is_running_)
    {
      stop ();
      start ();
    }
  }
}

void
pcl::RealSenseGrabber::disableTemporalFiltering ()
{
  enableTemporalFiltering (RealSense_None, 1);
}

const std::string&
pcl::RealSenseGrabber::getDeviceSerialNumber () const
{
  return (device_->getSerialNumber ());
}

std::vector<pcl::RealSenseGrabber::Mode>
pcl::RealSenseGrabber::getAvailableModes (bool only_depth) const
{
  std::vector<Mode> modes;
  PXCCapture::StreamType streams = only_depth
    ? PXCCapture::STREAM_TYPE_DEPTH
    : PXCCapture::STREAM_TYPE_DEPTH | PXCCapture::STREAM_TYPE_COLOR;
  for (int p = 0;; p++)
  {
    PXCCapture::Device::StreamProfileSet profiles = {};
    if (device_->getPXCDevice ().QueryStreamProfileSet (streams, p, &profiles) == PXC_STATUS_NO_ERROR)
    {
      if (!only_depth && profiles.depth.frameRate.max != profiles.color.frameRate.max)
        continue; // we need both streams to have the same framerate
      Mode mode;
      mode.fps = profiles.depth.frameRate.max;
      mode.depth_width = profiles.depth.imageInfo.width;
      mode.depth_height = profiles.depth.imageInfo.height;
      mode.color_width = profiles.color.imageInfo.width;
      mode.color_height = profiles.color.imageInfo.height;
      bool duplicate = false;
      for (size_t i = 0; i < modes.size (); ++i)
        duplicate |= modes[i] == mode;
      if (!duplicate)
        modes.push_back (mode);
    }
    else
    {
      break;
    }
  }
  return modes;
}

void
pcl::RealSenseGrabber::setMode (const Mode& mode, bool strict)
{
  if (mode == mode_requested_ && strict == strict_)
    return;
  mode_requested_ = mode;
  strict_ = strict;
  if (is_running_)
  {
    stop ();
    start ();
  }
}

#include <map>


void hokan(cv::Mat& src, cv::Mat& dst)
{
	int HEIGHT = src.rows;
	int WIDTH = src.cols;

	dst = cv::Mat(HEIGHT, WIDTH, CV_16UC1, cv::Scalar(pcl::RealSenseGrabber::VALUE_OFFSET));


	unsigned short* ps = (unsigned short*)(src.data);
	unsigned short* pd = (unsigned short*)(dst.data);

	for (int y = 0; y < HEIGHT; y++)
	{
		for (int x = 0; x < WIDTH; x++)
		{
			const int sw = 5;
			const int sh = 5;

			std::multimap<int, int> l;
			for (int sy = -sh; sy <= sh; sy++)
			{
				for (int sx = -sw; sx <= sw; sx++)
				{
					int x_ = x + sx; x_ = std::max(0, x_); x_ = std::min(WIDTH - 1, x_);
					int y_ = y + sy; y_ = std::max(0, y_); y_ = std::min(HEIGHT - 1, y_);

					if (ps[y_*WIDTH + x_] != pcl::RealSenseGrabber::VALUE_OFFSET)
					{
						l.insert(std::make_pair((abs(sy) + abs(sx)), ps[y_*WIDTH + x_]));
					}
				}
			}

			std::multimap<int, int>::iterator f = l.begin();
			std::multimap<int, int>::iterator s = l.begin(); s++;

			if (f != l.end())
			{
				if (s != l.end())
				{
					double	rf = (double)(s->first+1) / (f->first + s->first + 2);
					double rs = 1.0 - rf;
					int d = rf*f->second + rs*s->second;

					pd[y*WIDTH + x] = d;
				}
				else
				{
					pd[y*WIDTH + x] = s->second;
				}
			}
			else
			{
				pd[y*WIDTH + x] = pcl::RealSenseGrabber::VALUE_OFFSET;
			}
			
		}
	}
}


bool pcl::RealSenseGrabber::conv_camera_to_color(std::vector<PXCPoint3DF32>& cam_pos_list_mm, cv::Mat& imgx, cv::Mat& imgy, cv::Mat& imgz, bool vis)
{

	const int WIDTH = mode_selected_.depth_width;
	const int HEIGHT = mode_selected_.depth_height;

	
	if (WIDTH == 0 || HEIGHT == 0)
		return false;
	
	//printf("w h %d %d\n", WIDTH, HEIGHT);

	imgx = cv::Mat(HEIGHT, WIDTH, CV_16UC1, cv::Scalar(VALUE_OFFSET));
	imgy = cv::Mat(HEIGHT, WIDTH, CV_16UC1, cv::Scalar(VALUE_OFFSET));
	imgz = cv::Mat(HEIGHT, WIDTH, CV_16UC1, cv::Scalar(VALUE_OFFSET));

	PXCProjection* projection = device_->getPXCDevice().CreateProjection();

	int n = cam_pos_list_mm.size();

	std::vector<PXCPointF32> color_pos_list(n);
	pxcStatus st = projection->ProjectCameraToColor(n, &(cam_pos_list_mm[0]), &(color_pos_list[0]));
	//printf("cnv st %d\n", st);


	//画像にしたてる
	unsigned short* px = (unsigned short*)(imgx.data);
	unsigned short* py = (unsigned short*)(imgy.data);
	unsigned short* pz = (unsigned short*)(imgz.data);


	for(int i=0;i<n;i++)
	{
		int x = color_pos_list[i].x;	
		int y = color_pos_list[i].y;

		if (x == -1 || y == -1)	//projection failed
			continue;

		int aw = 0;// 5;
		int ah = 0;// 5;
		if (vis == true)
		{
			aw = ah = 5;
		}
		for (int ay = -ah; ay <= ah; ay++)
		{
			for (int ax = -aw; ax <= aw; ax++)
			{
				int x_ = x + ax; x_ = std::max(0, x_); x_ = std::min(WIDTH - 1, x_);
				int y_ = y + ay; y_ = std::max(0, y_); y_ = std::min(HEIGHT - 1, y_);


				//printf("%d %d %d\n", i, x_, y_);

				px[y_*WIDTH + x_] = cam_pos_list_mm[i].x + VALUE_OFFSET;
				py[y_*WIDTH + x_] = cam_pos_list_mm[i].y + VALUE_OFFSET;
				pz[y_*WIDTH + x_] = cam_pos_list_mm[i].z + VALUE_OFFSET;

			}
		}

	}

	//cv::Mat tx, ty, tz;
	//hokan(imgx, tx);
	//hokan(imgy, ty);
	//hokan(imgz, tz);

	//imgx = tx;
	//imgy = ty;
	//imgz = tz;

	projection->Release();

	return true;
}

void 
pcl::RealSenseGrabber::run ()
{
  const int WIDTH = mode_selected_.depth_width;
  const int HEIGHT = mode_selected_.depth_height;
  const int SIZE = WIDTH * HEIGHT;

  printf("w h %d %d\n", WIDTH, HEIGHT);

  PXCProjection* projection = device_->getPXCDevice ().CreateProjection ();
  PXCCapture::Sample sample;
  std::vector<PXCPoint3DF32> vertices (SIZE);
  createDepthBuffer ();

  while (is_running_)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr xyzrgba_cloud;

	ImageXYZRGBA_sepa	xyzrgba_sepa;

    pxcStatus status;
    if (need_xyzrgba_ || need_xyzrgba_sepa)
      status = device_->getPXCDevice ().ReadStreams (PXCCapture::STREAM_TYPE_DEPTH | PXCCapture::STREAM_TYPE_COLOR, &sample);
    else if(need_xyz_)
      status = device_->getPXCDevice ().ReadStreams (PXCCapture::STREAM_TYPE_DEPTH, &sample);
	else
	  status = device_->getPXCDevice().ReadStreams(PXCCapture::STREAM_TYPE_COLOR, &sample);


    uint64_t timestamp = pcl::getTime () * 1.0e+6;

	//printf("sts %d\n", status);
    switch (status)
    {
    case PXC_STATUS_NO_ERROR:
    {
      fps_mutex_.lock ();
      frequency_.event ();
      fps_mutex_.unlock ();

      /* We preform the following steps to convert received data into point clouds:
       *
       *   1. Push depth image to the depth buffer
       *   2. Pull filtered depth image from the depth buffer
       *   3. Project (filtered) depth image into 3D
       *   4. Fill XYZ point cloud with computed points
       *   5. Fill XYZRGBA point cloud with computed points
       *   7. Project color image into 3D
       *   6. Assign colors to points in XYZRGBA point cloud
       *
       * Steps 1-2 are skipped if temporal filtering is disabled.
       * Step 4 is skipped if there are no subscribers for XYZ clouds.
       * Steps 5-7 are skipped if there are no subscribers for XYZRGBA clouds. */

      if (temporal_filtering_type_ != RealSense_None)
      {
        PXCImage::ImageData data;
        sample.depth->AcquireAccess (PXCImage::ACCESS_READ, &data);
        std::vector<unsigned short> data_copy (SIZE);
        memcpy (data_copy.data (), data.planes[0], SIZE * sizeof (unsigned short));
        sample.depth->ReleaseAccess (&data);

        depth_buffer_->push (data_copy);

        sample.depth->AcquireAccess (PXCImage::ACCESS_WRITE, &data);
        unsigned short* d = reinterpret_cast<unsigned short*> (data.planes[0]);
        for (size_t i = 0; i < SIZE; i++)
          d[i] = (*depth_buffer_)[i];
        sample.depth->ReleaseAccess (&data);
      }

      projection->QueryVertices (sample.depth, vertices.data ());	//深度画像からxyzへ変換

      if (need_xyz_)
      {
        xyz_cloud.reset (new pcl::PointCloud<pcl::PointXYZ> (WIDTH, HEIGHT));
        xyz_cloud->header.stamp = timestamp;
        xyz_cloud->is_dense = false;
        for (int i = 0; i < SIZE; i++)
          convertPoint (vertices[i], xyz_cloud->points[i]);
      }

      if (need_xyzrgba_)
      {
        PXCImage::ImageData data;
        PXCImage* mapped = projection->CreateColorImageMappedToDepth (sample.depth, sample.color);
        mapped->AcquireAccess (PXCImage::ACCESS_READ, &data);
        uint32_t* d = reinterpret_cast<uint32_t*> (data.planes[0]);
        if (need_xyz_)
        {
          // We can fill XYZ coordinates more efficiently using pcl::copyPointCloud,
          // given that they were already computed for XYZ point cloud.
          xyzrgba_cloud.reset (new pcl::PointCloud<pcl::PointXYZRGBA>);
          pcl::copyPointCloud (*xyz_cloud, *xyzrgba_cloud);
          for (int i = 0; i < HEIGHT; i++)
          {
            pcl::PointXYZRGBA* cloud_row = &xyzrgba_cloud->points[i * WIDTH];
            uint32_t* color_row = &d[i * data.pitches[0] / sizeof (uint32_t)];
            for (int j = 0; j < WIDTH; j++)
              memcpy (&cloud_row[j].rgba, &color_row[j], sizeof (uint32_t));
          }
        }
        else
        {
          xyzrgba_cloud.reset (new pcl::PointCloud<pcl::PointXYZRGBA> (WIDTH, HEIGHT));
          xyzrgba_cloud->header.stamp = timestamp;
          xyzrgba_cloud->is_dense = false;
          for (int i = 0; i < HEIGHT; i++)
          {
            PXCPoint3DF32* vertices_row = &vertices[i * WIDTH];
            pcl::PointXYZRGBA* cloud_row = &xyzrgba_cloud->points[i * WIDTH];
            uint32_t* color_row = &d[i * data.pitches[0] / sizeof (uint32_t)];
            for (int j = 0; j < WIDTH; j++)
            {
              convertPoint (vertices_row[j], cloud_row[j]);
              memcpy (&cloud_row[j].rgba, &color_row[j], sizeof (uint32_t));
            }
          }
        }
        mapped->ReleaseAccess (&data);
        mapped->Release ();
      }

	  if (need_xyzrgba_sepa)
	  {
		  PXCImage::ImageData data, data_d;
		  PXCImage* mapped_depth = projection->CreateDepthImageMappedToColor(sample.depth, sample.color);
		  

		  sample.color->AcquireAccess(PXCImage::ACCESS_READ, &data);
		  mapped_depth->AcquireAccess(PXCImage::ACCESS_READ, &data_d);


		  xyzrgba_sepa.w = sample.color->QueryInfo().width;
		  xyzrgba_sepa.h = sample.color->QueryInfo().height;
		  xyzrgba_sepa.rgba_data.resize(xyzrgba_sepa.w*xyzrgba_sepa.h * sizeof(uint32_t));

		  uint32_t* d = reinterpret_cast<uint32_t*> (data.planes[0]);
		  for (int i = 0; i < xyzrgba_sepa.h; i++)
		  {
			  uint32_t* color_row = &d[i * data.pitches[0] / sizeof(uint32_t)];

			  for (int j = 0; j < xyzrgba_sepa.w; j++)
			  {
				  memcpy(&(xyzrgba_sepa.rgba_data[(i*xyzrgba_sepa.w + j) * sizeof(uint32_t)]), &color_row[j], sizeof(uint32_t));
			  }
		  }



		  if (data_d.format != PXCImage::PIXEL_FORMAT_DEPTH)
			  printf("error!!! format invalid!!");

		  //ProjectColorToCamera()に投げるデータの作成
		  int pnum = xyzrgba_sepa.w*xyzrgba_sepa.h;
		  std::vector<PXCPoint3DF32> color_pos_list(pnum);
		  uint16_t* dd = reinterpret_cast<uint16_t*> (data_d.planes[0]);
		  for (int i = 0, cntr = 0; i < xyzrgba_sepa.h; i++)
		  {
			  uint16_t* depth_row = &dd[i * data_d.pitches[0] / sizeof(uint16_t)];

			  for (int j = 0; j < xyzrgba_sepa.w; j++)
			  {
				  color_pos_list[cntr].x = j;
				  color_pos_list[cntr].y = i;

				  color_pos_list[cntr].z = depth_row[j];
				  
				  cntr++;
			  }
		  }

		  //変換
		  std::vector<PXCPoint3DF32> world_pos_list(pnum);
		  //printf("start conv\n");
		  projection->ProjectColorToCamera(pnum, &(color_pos_list[0]), &(world_pos_list[0]));
		  //printf("end conv\n");


		  //画像に成形
		  xyzrgba_sepa.x_data.resize(xyzrgba_sepa.w*xyzrgba_sepa.h);
		  xyzrgba_sepa.y_data.resize(xyzrgba_sepa.w*xyzrgba_sepa.h);
		  xyzrgba_sepa.z_data.resize(xyzrgba_sepa.w*xyzrgba_sepa.h);

		  for (int i = 0, cntr=0; i < xyzrgba_sepa.h; i++)
		  {
			  for (int j = 0; j < xyzrgba_sepa.w; j++)
			  {
				  xyzrgba_sepa.x_data[cntr] = world_pos_list[cntr].x;
				  xyzrgba_sepa.y_data[cntr] = world_pos_list[cntr].y;
				  xyzrgba_sepa.z_data[cntr] = world_pos_list[cntr].z;

				  cntr++;
			  }
		  }

		  sample.color->ReleaseAccess(&data);
		  mapped_depth->ReleaseAccess(&data_d);
		  mapped_depth->Release();

	  }


	  //if (need_xyzrgba_sepa)
	  //{
		 // PXCImage::ImageData data,data_d;
		 // //PXCImage* mapped = projection->CreateColorImageMappedToDepth(sample.depth, sample.color);
		 // //mapped->AcquireAccess(PXCImage::ACCESS_READ, &data);

		 // sample.color->AcquireAccess(PXCImage::ACCESS_READ, &data);
		 // sample.depth->AcquireAccess(PXCImage::ACCESS_READ, &data_d);
		 // 
		 // xyzrgba_sepa.w = sample.color->QueryInfo().width;
		 // xyzrgba_sepa.h = sample.color->QueryInfo().height;
		 // xyzrgba_sepa.rgba_data.resize(xyzrgba_sepa.w*xyzrgba_sepa.h * sizeof(uint32_t));

		 // uint32_t* d = reinterpret_cast<uint32_t*> (data.planes[0]);
		 // for (int i = 0; i < xyzrgba_sepa.h; i++)
		 // {
			//  uint32_t* color_row = &d[i * data.pitches[0] / sizeof(uint32_t)];

			//  for (int j = 0; j < xyzrgba_sepa.w; j++)
			//  {
			//	  memcpy(&(xyzrgba_sepa.rgba_data[(i*xyzrgba_sepa.w +j)* sizeof(uint32_t)]), &color_row[j], sizeof(uint32_t));
			//  }
		 // }



		 // if (data_d.format != PXCImage::PIXEL_FORMAT_DEPTH)
			//  printf("error!!! format invalid!!");

		 // xyzrgba_sepa.w = sample.depth->QueryInfo().width;
		 // xyzrgba_sepa.h = sample.depth->QueryInfo().height;
		 // xyzrgba_sepa.depth_data.resize(xyzrgba_sepa.w*xyzrgba_sepa.h);

		 // uint16_t* dd = reinterpret_cast<uint16_t*> (data_d.planes[0]);
		 // for (int i = 0; i < xyzrgba_sepa.h; i++)
		 // {
			//  uint16_t* depth_row = &dd[i * data_d.pitches[0] / sizeof(uint16_t)];

			//  for (int j = 0; j < xyzrgba_sepa.w; j++)
			//  {
			//	  memcpy(&(xyzrgba_sepa.depth_data[i*xyzrgba_sepa.w + j]), &depth_row[j], sizeof(uint16_t));
			//  }
		 // }

		 // sample.color->ReleaseAccess(&data);
		 // sample.depth->ReleaseAccess(&data_d);

	  //}

      if (need_xyzrgba_)
        point_cloud_rgba_signal_->operator () (xyzrgba_cloud);
      if (need_xyz_)
        point_cloud_signal_->operator () (xyz_cloud);
	  if (need_xyzrgba_sepa)
		  xyzrgba_sepa_signal_->operator () (xyzrgba_sepa);

      break;
    }
    case PXC_STATUS_DEVICE_LOST:
      THROW_IO_EXCEPTION ("failed to read data stream from PXC device: device lost");
    case PXC_STATUS_ALLOC_FAILED:
      THROW_IO_EXCEPTION ("failed to read data stream from PXC device: alloc failed");
    }
    sample.ReleaseImages ();
  }
  projection->Release ();
  RealSenseDevice::reset (device_);
}

float
pcl::RealSenseGrabber::computeModeScore (const Mode& mode)
{
  const float FPS_WEIGHT = 100000;
  const float DEPTH_WEIGHT = 1000;
  const float COLOR_WEIGHT = 1;
  int f = mode.fps - mode_requested_.fps;
  int dw = mode.depth_width - mode_requested_.depth_width;
  int dh = mode.depth_height - mode_requested_.depth_height;
  int cw = mode.color_width - mode_requested_.color_width;
  int ch = mode.color_height - mode_requested_.color_height;
  float penalty;
  penalty  = std::abs (FPS_WEIGHT * f * (mode_requested_.fps != 0));
  penalty += std::abs (DEPTH_WEIGHT * dw * (mode_requested_.depth_width != 0));
  penalty += std::abs (DEPTH_WEIGHT * dh * (mode_requested_.depth_height != 0));
  penalty += std::abs (COLOR_WEIGHT * cw * (mode_requested_.color_width != 0 && need_xyzrgba_));
  penalty += std::abs (COLOR_WEIGHT * ch * (mode_requested_.color_height != 0 && need_xyzrgba_));
  return penalty;
}

void
pcl::RealSenseGrabber::selectMode ()
{
  if (mode_requested_ == Mode ())
    mode_requested_ = Mode (30, 640, 480, 640, 480);
  float best_score = std::numeric_limits<float>::max ();
  std::vector<Mode> modes = getAvailableModes (!need_xyzrgba_);
  for (size_t i = 0; i < modes.size (); ++i)
  {
    Mode mode = modes[i];
    float score = computeModeScore (mode);
    if (score < best_score)
    {
      best_score = score;
      mode_selected_ = mode;
    }
  }
  if (strict_ && best_score > 0)
    THROW_IO_EXCEPTION ("PXC device does not support requested mode");
}

void
pcl::RealSenseGrabber::createDepthBuffer ()
{
  size_t size = mode_selected_.depth_width * mode_selected_.depth_height;
  switch (temporal_filtering_type_)
  {
  case RealSense_None:
  {
    depth_buffer_.reset (new pcl::io::SingleBuffer<unsigned short> (size));
    break;
  }
  case RealSense_Median:
  {
    depth_buffer_.reset (new pcl::io::MedianBuffer<unsigned short> (size, temporal_filtering_window_size_));
    break;
  }
  case RealSense_Average:
  {
    depth_buffer_.reset (new pcl::io::AverageBuffer<unsigned short> (size, temporal_filtering_window_size_));
    break;
  }
  }
}
