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

#include "stdafx.h"

#include "../src/opencv_include.h"

#include "../src/rs_grabber/real_sense_grabber.h"

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl_conversions/pcl_conversions.h"

#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"


#pragma comment(lib,"cpp_common.lib")
#pragma comment(lib,"console_bridge.lib")
#pragma comment(lib,"roscpp.lib")
#pragma comment(lib,"roscpp_serialization.lib")
#pragma comment(lib,"rostime.lib")
#pragma comment(lib,"rosconsole.lib")


#include "apc2016\ItemPoseEst.h"
#include "apc2016\ItemPoseEstAddData.h"
#include "apc2016\Segmentation.h"
#include "apc2016\CoordinateTransform.h"
#include "apc2016\CameraCoordinateTransform.h"
#include "apc2016\CameraCoordinateTransformAll.h"

#include "../src/load_item_list.h"
#include "../src/pcl_funcs/pose_est_funcs.h"
#include "../src/pcl_funcs/filter_funcs.h"
#include "../src/pcl_funcs/plane_funcs.h"
#include "../src/pcl_funcs/file_funcs.h"


#include <pcl/visualization/cloud_viewer.h>

char ROS_MASTER_STR[100] = "\0";
char WHERE[100] = "\0";

ros::ServiceClient cl_segm;
ros::ServiceClient cl_c2g;
ros::ServiceClient cl_c2ga;
ros::ServiceClient cl_g2b;

item_list	items;

std::vector<pcl::PointCloud<pcl::PointXYZRGB> >	model_list;
bool prepare_model_list(int itmid, std::vector<pcl::PointCloud<pcl::PointXYZRGB> >&	model_list);

pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcld(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcld_tmp(new pcl::PointCloud<pcl::PointXYZRGB>);

pcl::PointCloud<pcl::PointXYZRGB>::Ptr model_aligned(new pcl::PointCloud<pcl::PointXYZRGB>);

pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcld_view(new pcl::PointCloud<pcl::PointXYZRGB>);

//pcl::visualization::PCLVisualizer visu("target");
pcl::visualization::CloudViewer* pv;

boost::recursive_mutex mtx;



void convert_pencilcup(std::vector<std::vector<int> >& seg_map, int itmid, apc2016::Segmentation& segs, apc2016::ItemPoseEstAddDataRequest& req)
{


	const int sw = segs.response.score_map_width;
	const int sh = segs.response.score_map_height;

	//convert pixels to pcl data
	cv_bridge::CvImagePtr pimg = cv_bridge::toCvCopy(segs.response.realsense_rgb_img, sensor_msgs::image_encodings::BGR8);
	cv_bridge::CvImagePtr pimgx = cv_bridge::toCvCopy(segs.response.realsense_x_img, sensor_msgs::image_encodings::MONO16);
	cv_bridge::CvImagePtr pimgy = cv_bridge::toCvCopy(segs.response.realsense_y_img, sensor_msgs::image_encodings::MONO16);
	cv_bridge::CvImagePtr pimgz = cv_bridge::toCvCopy(segs.response.realsense_z_img, sensor_msgs::image_encodings::MONO16);

	cv::imshow("rgb", pimg->image);

	int w = pimgx->image.cols;
	int h = pimgx->image.rows;

	double wrate = (double)sw / w;
	double hrate = (double)sh / h;


	unsigned char* pi = (unsigned char*)(pimg->image.data);
	unsigned short* px = (unsigned short*)(pimgx->image.data);
	unsigned short* py = (unsigned short*)(pimgy->image.data);
	unsigned short* pz = (unsigned short*)(pimgz->image.data);


	pcld->is_dense = false;




	int cnt = 0;



	printf("c2ga\n");
	for (int y = 0; y < h; y +=5)
	{
		//printf("\r%d/%d", y, h);
		//fflush(stdout);
		for (int x = 0; x < w; x +=5)
		{
			if (seg_map[(int)(y*hrate)][(int)(x*wrate)] == itmid)
			{


				float x_ = 0;
				float y_ = -(x - w / 2);
				float z_ = -y;

				//printf("%f %f %f\n", x_, y_, z_);															


				pcl::PointXYZRGB pt;

				pt.x = x_ / 1000;
				pt.y = y_ / 1000;
				pt.z = z_ / 1000;

				pt.b = pi[y*w * 3 + x * 3 + 0];
				pt.g = pi[y*w * 3 + x * 3 + 1];
				pt.r = pi[y*w * 3 + x * 3 + 2];

				pcld->points.push_back(pt);

				cnt++;
				//if (cnt > 100)
				//	break;

			}
		}

		//if (cnt > 100)
		//	break;

	}


	pcld->width = pcld->points.size();
	pcld->height = 1;
	pcld->is_dense = false;
}


void convert_realsense_pcl_data2(std::vector<std::vector<int> >& seg_map, int itmid, apc2016::Segmentation& segs, apc2016::ItemPoseEstAddDataRequest& req)
{


	const int sw = segs.response.score_map_width;
	const int sh = segs.response.score_map_height;

	//convert pixels to pcl data
	cv_bridge::CvImagePtr pimg = cv_bridge::toCvCopy(segs.response.realsense_rgb_img, sensor_msgs::image_encodings::BGR8);
	cv_bridge::CvImagePtr pimgx = cv_bridge::toCvCopy(segs.response.realsense_x_img, sensor_msgs::image_encodings::MONO16);
	cv_bridge::CvImagePtr pimgy = cv_bridge::toCvCopy(segs.response.realsense_y_img, sensor_msgs::image_encodings::MONO16);
	cv_bridge::CvImagePtr pimgz = cv_bridge::toCvCopy(segs.response.realsense_z_img, sensor_msgs::image_encodings::MONO16);

	cv::imshow("rgb", pimg->image);

	int w = pimgx->image.cols;
	int h = pimgx->image.rows;

	double wrate = (double)sw / w;
	double hrate = (double)sh / h;


	unsigned char* pi = (unsigned char*)(pimg->image.data);
	unsigned short* px = (unsigned short*)(pimgx->image.data);
	unsigned short* py = (unsigned short*)(pimgy->image.data);
	unsigned short* pz = (unsigned short*)(pimgz->image.data);


	pcld->is_dense = false;




	int cnt = 0;


	pcld_tmp->clear();

	printf("c2ga\n");
	for (int y = 0; y < h; y++)
	{
		//printf("\r%d/%d", y, h);
		//fflush(stdout);
		for (int x = 0; x < w; x++)
		{
			if (seg_map[(int)(y*hrate)][(int)(x*wrate)] == itmid)
			{
				

				float x_ = (float)(px[y*w + x] - pcl::RealSenseGrabber::VALUE_OFFSET);	//img x->y global
				float y_ = (float)(py[y*w + x] - pcl::RealSenseGrabber::VALUE_OFFSET);	//img y->z
				float z_ = (float)(pz[y*w + x] - pcl::RealSenseGrabber::VALUE_OFFSET);	//img z->x

				//printf("%f %f %f\n", x_, y_, z_);

				pcl::PointXYZRGB pt;

				pt.x = x_;
				pt.y = y_;
				pt.z = z_;

				pt.b = pi[y*w * 3 + x * 3 + 0];
				pt.g = pi[y*w * 3 + x * 3 + 1];
				pt.r = pi[y*w * 3 + x * 3 + 2];

				pcld_tmp->points.push_back(pt);

				cnt++;
				//if (cnt > 100)
				//	break;

			}
		}

		//if (cnt > 100)
		//	break;

	}

	pcl::copyPointCloud(*pcld_tmp, *pcld_view);
	printf("pv s %d\n", pcld_view->size());

	for (int p = 0; p < pcld_view->size(); p++)
	{
		pcld_view->points[p].x = pcld_view->points[p].x / 1000.0;	
		pcld_view->points[p].y = pcld_view->points[p].y / 1000.0;	//mm -> m
		pcld_view->points[p].z = pcld_view->points[p].z / 1000.0;

		int r = pcld_view->points[p].r + 50; r = (r > 255) ? 255 : r;
		int g = pcld_view->points[p].g + 50; g = (g > 255) ? 255 : g;
		int b = pcld_view->points[p].b + 50; b = (b > 255) ? 255 : b;

	}


	pcl::console::print_highlight("Downsampling...\n");
	pcl::VoxelGrid<pcl::PointXYZRGB> grid;
	const float leaf = 5;	//mm
	grid.setLeafSize(leaf, leaf, leaf);
	grid.setInputCloud(pcld_tmp);
	printf("object p num:%d\n", pcld_tmp->points.size());
	grid.filter(*pcld_tmp);
	printf("object p num:%d\n", pcld_tmp->points.size());



	apc2016::CameraCoordinateTransformAll ctcg;
	ctcg.request.xyzwpr.linear.x = req.x;
	ctcg.request.xyzwpr.linear.y = req.y;
	ctcg.request.xyzwpr.linear.z = req.z;
	ctcg.request.xyzwpr.angular.x = req.ax;
	ctcg.request.xyzwpr.angular.y = req.ay;
	ctcg.request.xyzwpr.angular.z = req.az;

	for (int i = 0; i < pcld_tmp->size(); i++)
	{
		ctcg.request.x.push_back(pcld_tmp->points[i].x);
		ctcg.request.y.push_back(pcld_tmp->points[i].y);
		ctcg.request.z.push_back(pcld_tmp->points[i].z);
	}

	bool ret = cl_c2ga.call(ctcg);
	if (ret == false) { printf("camera2global_all service failed.\n"); return; }
				
	printf("c2ga finished\n");

	int prev_id = pcld->size();
	int nx = ctcg.response.x.size();
	pcld->resize(prev_id + nx);
	for (int i = 0; i < nx; i++)
	{
		//printf("\r%d/%d", i, n);
		//fflush(stdout);

		pcld->points[i + prev_id].x = ctcg.response.x[i] / 1000.0;	
		pcld->points[i + prev_id].y = ctcg.response.y[i] / 1000.0;	//mm -> m
		pcld->points[i + prev_id].z = ctcg.response.z[i] / 1000.0;

		//global x->z
		//y->x
		//z->y

		//printf("%f %f %f\n", pcld->points[i].x, pcld->points[i].y, pcld->points[i].z);
	}
	printf("\n");
	pcld->width = pcld->points.size();
	pcld->height = 1;
	pcld->is_dense = false;
}

void convert_realsense_pcl_data(std::vector<std::vector<int> >& seg_map, int itmid, apc2016::Segmentation& segs, apc2016::ItemPoseEstAddDataRequest& req)
{


	const int sw = segs.response.score_map_width;
	const int sh = segs.response.score_map_height;

	//convert pixels to pcl data
	cv_bridge::CvImagePtr pimg = cv_bridge::toCvCopy(segs.response.realsense_rgb_img, sensor_msgs::image_encodings::BGR8);
	cv_bridge::CvImagePtr pimgx = cv_bridge::toCvCopy(segs.response.realsense_x_img, sensor_msgs::image_encodings::MONO16);
	cv_bridge::CvImagePtr pimgy = cv_bridge::toCvCopy(segs.response.realsense_y_img, sensor_msgs::image_encodings::MONO16);
	cv_bridge::CvImagePtr pimgz = cv_bridge::toCvCopy(segs.response.realsense_z_img, sensor_msgs::image_encodings::MONO16);

	cv::imshow("rgb", pimg->image);

	int w = pimgx->image.cols;
	int h = pimgx->image.rows;

	double wrate = (double)sw / w;
	double hrate = (double)sh / h;


	unsigned char* pi = (unsigned char*)(pimg->image.data);
	unsigned short* px = (unsigned short*)(pimgx->image.data);
	unsigned short* py = (unsigned short*)(pimgy->image.data);
	unsigned short* pz = (unsigned short*)(pimgz->image.data);


	pcld->is_dense = false;

	apc2016::CameraCoordinateTransform ctcg;
	ctcg.request.xyzwpr.linear.x = req.x;
	ctcg.request.xyzwpr.linear.y = req.y;
	ctcg.request.xyzwpr.linear.z = req.z;
	ctcg.request.xyzwpr.angular.x = req.ax;
	ctcg.request.xyzwpr.angular.y = req.ay;
	ctcg.request.xyzwpr.angular.z = req.az;

	ctcg.request.point.angular.x = 0;	//not use angle
	ctcg.request.point.angular.y = 0;
	ctcg.request.point.angular.z = 0;

	printf("c2g\n");
	for (int y = 0; y < h; y++)
	{
		printf("\r%d/%d", y, h);
		fflush(stdout);
		for (int x = 0; x < w; x++)
		{
			if (seg_map[(int)(y*hrate)][(int)(x*wrate)] == itmid)
			{
				pcl::PointXYZRGB pt;

				ctcg.request.point.linear.y = (float)(px[y*w + x] - 30000);	//img x->y global
				ctcg.request.point.linear.z = (float)(py[y*w + x] - 30000);	//y->z
				ctcg.request.point.linear.x = (float)(pz[y*w + x] - 30000);	//z->x

				printf("%f %f %f\n", ctcg.request.point.linear.x, ctcg.request.point.linear.y, ctcg.request.point.linear.z);

				bool ret = cl_c2g.call(ctcg);
				if (ret == false) { printf("camera2global service failed.\n"); }

				printf("%f %f %f\n", ctcg.response.point.linear.x, ctcg.response.point.linear.y, ctcg.response.point.linear.z);

				pt.z = ctcg.response.point.linear.x / 1000.0;	
				pt.x = ctcg.response.point.linear.y / 1000.0;	//mm -> m
				pt.y = ctcg.response.point.linear.z / 1000.0;
				//global x->z
				//y->x
				//z->y

				pt.b = pi[y*w * 3 + x * 3 + 0];
				pt.g = pi[y*w * 3 + x * 3 + 1];
				pt.r = pi[y*w * 3 + x * 3 + 2];

				pcld->points.push_back(pt);

			}

		}
	}
	pcld->width = pcld->points.size();
	pcld->height = 1;
	pcld->is_dense = false;
}

void convert_fx8_pcl_data(std::vector<std::vector<int> >& seg_map, int itmid, apc2016::Segmentation& segs, apc2016::ItemPoseEstAddDataRequest& req)
{
	const int sw = segs.response.score_map_width;
	const int sh = segs.response.score_map_height;

	//convert pixels to pcl data
	cv_bridge::CvImagePtr pimg = cv_bridge::toCvCopy(segs.response.realsense_rgb_img, sensor_msgs::image_encodings::BGR8);
	cv_bridge::CvImagePtr pimgx = cv_bridge::toCvCopy(segs.response.fx8_x_img, sensor_msgs::image_encodings::MONO16);
	cv_bridge::CvImagePtr pimgy = cv_bridge::toCvCopy(segs.response.fx8_y_img, sensor_msgs::image_encodings::MONO16);
	cv_bridge::CvImagePtr pimgz = cv_bridge::toCvCopy(segs.response.fx8_z_img, sensor_msgs::image_encodings::MONO16);

	cv::imshow("rgb", pimg->image);

	int w = pimgx->image.cols;
	int h = pimgx->image.rows;

	double wrate = (double)sw / w;
	double hrate = (double)sh / h;

	unsigned short* px = (unsigned short*)(pimgx->image.data);
	unsigned short* py = (unsigned short*)(pimgy->image.data);
	unsigned short* pz = (unsigned short*)(pimgz->image.data);


	pcld->is_dense = false;


	apc2016::CameraCoordinateTransform ctcg;
	ctcg.request.xyzwpr.linear.x = req.x;
	ctcg.request.xyzwpr.linear.y = req.y;
	ctcg.request.xyzwpr.linear.z = req.z;
	ctcg.request.xyzwpr.angular.x = req.ax;
	ctcg.request.xyzwpr.angular.y = req.ay;
	ctcg.request.xyzwpr.angular.z = req.az;

	ctcg.request.point.angular.x = 0;	
	ctcg.request.point.angular.y = 0;
	ctcg.request.point.angular.z = 0;

	printf("c2g\n");
	for (int y = 0; y < h; y++)
	{
		printf("\r%d/%d", y, h);
		fflush(stdout);
		for (int x = 0; x < w; x++)
		{
			if (seg_map[(int)(y*hrate)][(int)(x*wrate)] == itmid)
			{
				if (px[y*w + x] != 30000)
				{

					pcl::PointXYZRGB pt;

					ctcg.request.point.linear.y = (float)(px[y*w + x] - 30000);	//img x->y global
					ctcg.request.point.linear.z = (float)(py[y*w + x] - 30000);	//y->z
					ctcg.request.point.linear.x = (float)(pz[y*w + x] - 30000);	//z->x

					bool ret = cl_c2g.call(ctcg);
					if (ret == false) { printf("camera2global service failed.\n"); }

					pt.z = ctcg.response.point.linear.x / 1000.0;	
					pt.x = ctcg.response.point.linear.y / 1000.0;	//mm -> m
					pt.y = ctcg.response.point.linear.z / 1000.0;
					//global x->z
					//y->x
					//z->y

					pt.b = 255;
					pt.g = 255;
					pt.r = 0;

					pcld->points.push_back(pt);

				}
			}

		}
	}
	pcld->width = pcld->points.size();
	pcld->height = 1;
	pcld->is_dense = false;
}

bool addDataSvcb(apc2016::ItemPoseEstAddDataRequest& req, apc2016::ItemPoseEstAddDataResponse& res)
{

	boost::lock_guard<boost::recursive_mutex> lock(mtx);

	if (items.name_to_id.find(req.item) == items.name_to_id.end())
	{
		printf("invalid item name in item_pose_est_node\n");
		return 0;
	}
	int itmid = items.name_to_id[req.item];

	printf("%s %d\n", req.item.c_str(), itmid);

	if (req.is_first_data == true)
	{
		//初期化
		pcld->clear();
		pcld_tmp->clear();
		pcld_view->clear();
		model_aligned->clear();
	}
	

	//segmentation map
	apc2016::Segmentation segs;

	segs.request.items.push_back("tmp");

	printf("seg\n");
	bool ret = cl_segm.call(segs);
	if (ret == false)
	{
		printf("seg servie failed.\n");
		return true;
	}
	printf("seg finish\n");

	const int sw = segs.response.score_map_width;
	const int sh = segs.response.score_map_height;

	std::vector<float>& sm = segs.response.score_map;

	std::vector<int>	tmpx(sw, 0);
	std::vector<std::vector<int> >	seg_map(sh, tmpx);



	cv_bridge::CvImagePtr pimg = cv_bridge::toCvCopy(segs.response.realsense_rgb_img, sensor_msgs::image_encodings::BGR8);
	cv::Mat imgseg = pimg->image;

	cv::Mat seg(sh, sw, CV_8UC1);

	int nitm = items.list.size();
	for (int y = 0; y < sh; y++)
	{
		for (int x = 0; x < sw; x++)
		{
			int max_id = 0;
			float max_s = -10.0;
			for (int ci = 0; ci <= nitm; ci++)
			{
				int offset = sh*sw*ci + y*sw + x;

				float s = sm[offset];
				if (s > max_s)
				{
					max_s = s;
					max_id = ci;
				}
			}
			seg_map[y][x] = max_id;

			seg.data[y*sw + x] = (max_id == itmid) ? 255 : max_id;


			
		}
	}

	int h = imgseg.rows;
	int w = imgseg.cols;

	double wrate = (double)sw / w;
	double hrate = (double)sh / h;
	for (int y = 0; y < h; y++)
	{
		for (int x = 0; x < w; x++)
		{
			if (seg_map[y*hrate][x*wrate] == itmid)
			{
				unsigned char b = imgseg.data[y*w * 3 + x * 3 + 0];
				unsigned char g = imgseg.data[y*w * 3 + x * 3 + 1];
				unsigned char r = imgseg.data[y*w * 3 + x * 3 + 2];
				imgseg.data[y*w * 3 + x * 3 + 0] = std::min((int)(b + 50), 255);
				imgseg.data[y*w * 3 + x * 3 + 1] = std::min((int)(g + 50), 255);
				imgseg.data[y*w * 3 + x * 3 + 2] = std::min((int)(r + 50), 255);
			}
		}
	}
	cv::imshow("seg", seg);
	cv::imshow("img+seg", imgseg);
	cv::waitKey(1);



	switch (itmid)
	{
	case 17:	//dumbbell
		//convert_realsense_pcl_data(seg_map, itmid, segs, req);
		convert_realsense_pcl_data2(seg_map, itmid, segs, req);
		break;

	case 18:	//pencil cup
		convert_pencilcup(seg_map, itmid, segs, req);
		break;

	case 25:	//水ボトル
		convert_fx8_pcl_data(seg_map, itmid, segs, req);
		break;
	default:
		convert_realsense_pcl_data(seg_map, itmid, segs, req);
		break;
	}


	if (pcld->size() == 0)
	{
		printf("no obj included.\nexit.\n");
		return true;
	}

	pv->showCloud(pcld_view);

	printf("add finished\n");

	return true;
}

bool pose_est_funcs1(int itmid, apc2016::ItemPoseEstResponse& res)
{

	if (prepare_model_list(itmid, model_list) == false)
	{
		return false;
	}


	POSE_EST_PARAMS p;
	bool ret_ = p.read_params("pose_est_params.txt");
	if (ret_ == false)
	{
		printf("exit this proc.\n");
		return true;
	}
	printf("read param ok.\n");
	p.print_params();

	double cx, cy, cz, ax, ay, az;
	bool ret = pose_est_(model_list[itmid], pcld, model_aligned, cx, cy, cz, ax, ay, az, p);
	if (ret == false)
	{
		res.success = false;
		res.no_item = false;
	}


	for (int i = 0; i < pcld->size(); i++)
	{
		double r_ = pcld->points[i].r*1.5;
		pcld->points[i].r = r_ > 255 ? 255 : r_;
		pcld->points[i].g *= 0.5;
		pcld->points[i].b *= 0.5;
	}
	pcld_view->points.insert(pcld_view->points.end(), pcld->points.begin(), pcld->points.end());
	pcld_view->width += pcld->points.size();

	pcld_view->points.insert(pcld_view->points.end(), model_list[itmid].points.begin(), model_list[itmid].points.end());
	pcld_view->width += model_list[itmid].points.size();

	pcld_view->points.insert(pcld_view->points.end(), model_aligned->points.begin(), model_aligned->points.end());
	pcld_view->width += model_aligned->points.size();

	printf("model_aligned num %d\n", model_aligned->points.size());

	pv->showCloud(pcld_view);

	res.x = cx * 1000;
	res.y = cy * 1000;
	res.z = cz * 1000;
	res.ax = ax;
	res.ay = ay;
	res.az = az;

	res.success = true;
	res.no_item = false;
}

void view_and_stop_pcld()
{
	pcld_view->clear();

	pcld_view->points.insert(pcld_view->points.end(), pcld->points.begin(), pcld->points.end());
	pcld_view->width += pcld->points.size();
	pv->showCloud(pcld_view);

	printf("show and stop\n");
	getchar();

}
bool pose_est_funcs2(int itmid, apc2016::ItemPoseEstResponse& res)
{


	//segmentation
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> out_cld_list;
	extract_objects(
		pcld,
		100,
		100000,
		0.01,
		out_cld_list);


	//centroid
	std::vector<Eigen::Vector4f> ctr_list(out_cld_list.size());
	for (int i = 0; i < out_cld_list.size(); i++)
	{
		pcl::compute3DCentroid(*out_cld_list[i], ctr_list[i]);
	}

	double min_dist = -1;
	int min_id = -1;
	for (int i = 0; i < ctr_list.size(); i++)
	{
		double z = ctr_list[i].z();
		printf("%d z:%lf\n", i, z);
		if (min_id == -1 || z < min_dist)
		{
			min_id = i;
			min_dist = z;
		}
	}

	*pcld = *out_cld_list[min_id];

	//view_and_stop_pcld();


	res.x = ctr_list[min_id].x();
	res.y = ctr_list[min_id].y();
	res.z = ctr_list[min_id].z();

	res.success = true;
	res.no_item = false;

	return true;
}

bool pose_est_funcs3(int itmid, apc2016::ItemPoseEstResponse& res)
{
	if (pcld->size() == 0)
	{
		res.no_item = true;
		res.success = true;

		return true;
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr t(new pcl::PointCloud<pcl::PointXYZRGB>);

	extract_bigger_objects(
		pcld,
		10,
		100000,
		0.1,
		t);


	int n = t->size();
	printf("%d\n", n);
	cv::Mat dt(n, 3, CV_64FC1);

	for (int i = 0; i < n; i++)
	{
		double *src = dt.ptr<double>(i);

		src[0] = t->points[i].x;
		src[1] = t->points[i].y;
		src[2] = t->points[i].z;

		//printf("%f %f %f\n", src[0], src[1], src[2]);
	}
	cv::PCA pca(dt, cv::Mat(), CV_PCA_DATA_AS_ROW, 0);	//0:all vector

	cv::Mat v1 = pca.eigenvectors.row(0);
	cv::Mat v2 = pca.eigenvectors.row(1);
	cv::Mat v3 = pca.eigenvectors.row(2);

	printf("m %lf %lf %lf\n", pca.mean.at<double>(0), pca.mean.at<double>(1), pca.mean.at<double>(2));
	printf("v %lf %lf %lf\n", v1.at<double>(0), v1.at<double>(1), v1.at<double>(2));

	res.x = pca.mean.at<double>(0) * 1000;
	res.y = pca.mean.at<double>(1) * 1000;
	res.z = pca.mean.at<double>(2) * 1000;
	res.ax = v1.at<double>(0);
	res.ay = v1.at<double>(1);
	res.az = v1.at<double>(2);

	res.no_item = false;
	res.success = true;

	return true;
}


bool pose_est_funcs_pencilcup(int itmid, apc2016::ItemPoseEstResponse& res)
{
	if (pcld->size() == 0)
	{
		res.no_item = true;
		res.success = true;

		return true;
	}



	int n = pcld->size();
	printf("%d\n", n);
	cv::Mat dt(n, 3, CV_64FC1);

	for (int i = 0; i < n; i++)
	{
		double *src = dt.ptr<double>(i);

		src[0] = pcld->points[i].x;
		src[1] = pcld->points[i].y;
		src[2] = pcld->points[i].z;

		//printf("%f %f %f\n", src[0], src[1], src[2]);
	}
	cv::PCA pca(dt, cv::Mat(), CV_PCA_DATA_AS_ROW, 0);	//0:all vector

	cv::Mat v1 = pca.eigenvectors.row(0);
	cv::Mat v2 = pca.eigenvectors.row(1);
	cv::Mat v3 = pca.eigenvectors.row(2);

	printf("m %lf %lf %lf\n", pca.mean.at<double>(0), pca.mean.at<double>(1), pca.mean.at<double>(2));
	printf("v %lf %lf %lf\n", v1.at<double>(0), v1.at<double>(1), v1.at<double>(2));

	res.x = pca.mean.at<double>(0) * 1000;
	res.y = pca.mean.at<double>(1) * 1000;
	res.z = pca.mean.at<double>(2) * 1000;
	res.ax = v1.at<double>(0);
	res.ay = v1.at<double>(1);
	res.az = v1.at<double>(2);

	res.no_item = false;
	res.success = true;

	return true;
}


bool svcb(apc2016::ItemPoseEstRequest& req, apc2016::ItemPoseEstResponse& res)
{
	boost::lock_guard<boost::recursive_mutex> lock(mtx);

	if (items.name_to_id.find(req.item) == items.name_to_id.end())
	{
		printf("invalid item name in item_pose_est_node\n");
		return 0;
	}
	int itmid = items.name_to_id[req.item];


	printf("p size :%d\n", pcld->size());
	//if (pcld->size() < 10)
	if( (itmid == 17 && pcld->size() < 500) || pcld->size() < 10)
	{
		res.success = true;
		res.no_item = true;

		printf("no item\n");
		return true;
	}


	//save_time_fname(*pcld);

	switch (itmid)
	{
	case 17:	//dumbbell
		//pose_est_funcs1(itmid, res);
		pose_est_funcs3(itmid, res);
		break;

	case 18:	//pencil cup
		pose_est_funcs_pencilcup(itmid, res);
		break;

	case 25:	//水ボトル
		//pose_est_funcs2(itmid, res);
		pose_est_funcs3(itmid, res);
		break;
	default:

		break;
	}


	printf("x:%f y:%f z:%f ax:%f ay:%f az:%f\n",res.x, res.y, res.z, res.ax, res.ay, res.az);

	return true;
}

int _tmain(int argc, _TCHAR * argv[])
{

	if (argc < 3)
	{
		printf("not enough arguments, ROS_MASTER_URI, where(left or right)\n");
		return 0;
	}


	strcpy(ROS_MASTER_STR, argv[1]);
	strcpy(WHERE, argv[2]);


	printf("<item_pose_est_node conf>\n");
	printf("ROS_MASTER_URI=%s\n", ROS_MASTER_STR);
	printf("%s\n", WHERE);


	//ROS関連
	//putenv("ROS_MASTER_URI=http://192.168.0.1:11311");
	char tmp[100];
	sprintf(tmp, "ROS_MASTER_URI=%s", ROS_MASTER_STR);
	putenv(tmp);
	fprintf(stdout, "ROS_MASTER_URI=%s\n", getenv("ROS_MASTER_URI"));


	bool ret = LoadItemList("item_data.csv", items);
	if (ret == false)
	{
		printf("can't load item_data.csv");
		return 0;
	}

	model_list.resize(items.list.size()+1);	//backgroundとindex対応


	ros::init(argc, argv, std::string("item_pose_est_node_") + WHERE);

	ros::NodeHandle nh;

	ros::ServiceServer	sv = nh.advertiseService(std::string("item_pose_est_") + WHERE, svcb);
	ros::ServiceServer	sv2 = nh.advertiseService(std::string("item_pose_est_adddata_") + WHERE, addDataSvcb);

	


	cl_segm = nh.serviceClient<apc2016::Segmentation>(std::string("img_segm_") + WHERE);
	printf("waiting service : %s\n", cl_segm.getService().c_str());
	cl_segm.waitForExistence();

	if (std::string(WHERE) == "right")
	{
		cl_c2g = nh.serviceClient<apc2016::CameraCoordinateTransform>(std::string("camera2global_right"));
		cl_c2ga = nh.serviceClient<apc2016::CameraCoordinateTransformAll>(std::string("camera2global_right_all"));
	}
	else
	{
		cl_c2g = nh.serviceClient<apc2016::CameraCoordinateTransform>(std::string("camera2global"));
		cl_c2ga = nh.serviceClient<apc2016::CameraCoordinateTransformAll>(std::string("camera2global_all"));
	}
	printf("waiting service : %s\n", cl_c2g.getService().c_str());
	cl_c2g.waitForExistence();
	cl_c2ga.waitForExistence();

	cl_g2b = nh.serviceClient<apc2016::CoordinateTransform>(std::string("global2bin"));
	printf("waiting service : %s\n", cl_g2b.getService().c_str());
	cl_g2b.waitForExistence();
	
	printf("service ok.\n");


	pv = new pcl::visualization::CloudViewer("view");
	

	while (ros::isShuttingDown() == false)
	{
		cv::waitKey(10);
		ros::spinOnce();

		{
			boost::lock_guard<boost::recursive_mutex> lock(mtx);

			//visu.spinOnce();
		}

		Sleep(100);
	}
	

	return 0;
}

bool prepare_model_list(int itmid, std::vector<pcl::PointCloud<pcl::PointXYZRGB> >&	model_list)
{
	if (model_list[itmid].points.size() == 0)
	{
		std::string	fname;
		switch (itmid)
		{
		case 17:	//dumbbel
			//fname = "17_fitness_gear_3lb_dumbbell_new.pcd";
			fname = "dumbbell2_100k_m.pcd";
			break;
		case 18:
			fname = "18_rolodex_jumbo_pencil_cup.pcd";
			break;

		case 25:	//水ボトル
			fname = "17_fitness_gear_3lb_dumbbell_10000pt.pcd";
			break;

		default:
			fname = "xx.pcd";
			break;

		}

		if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(fname, model_list[itmid]) < 0)
		{
			printf("can't open model file %s\n", fname.c_str());
			return false;
		}
	}


	return true;
}