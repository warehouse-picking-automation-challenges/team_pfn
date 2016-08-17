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

#include "../src\rs_grabber/real_sense_grabber.h"

#include "../src/opencv_include.h"

#include <algorithm>
#include <cmath>

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




#include <pcl/visualization/cloud_viewer.h>
#include <pcl/segmentation/sac_segmentation.h>  
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>

#include "../src/pcl_cfg.h"

#include "apc2016\XYZImage.h"


const bool PUB_PCL = false;


bool IMG_FLIP = false;


bool exit_flg = false;

boost::recursive_mutex mtx;

pcl::PointCloud<pcl::PointXYZ>::Ptr fx8_cld(new pcl::PointCloud<pcl::PointXYZ>);

ros::Publisher fx8pcpub;
ros::Publisher fx8xpub;
ros::Publisher fx8ypub;
ros::Publisher fx8zpub;
ros::Publisher fx8xyzpub;
sensor_msgs::PointCloud2	sm_fx8cld;
pcl::RealSenseGrabber* rsg;

char ROS_MASTER_STR[100] = "\0";
char WHERE[100] = "\0";
char DEVICEID[50] = "\0";

bool hokan_window_flg = false;
int prev_key = -1;


template<typename T>
void gaussian_interpolation(
	//const cv::Mat_<T>& src, cv::Mat_<T>& dst, int ksize, T threshold, T beta) {
	const cv::Mat_<T>& src, cv::Mat_<T>& dst, int ksize, T offset_val, float beta) {
	assert(src.size() == dst.size());
	const int rows = src.rows;
	const int cols = src.cols;
	const int K = ksize / 2;
#ifdef _OPENMP
#pragma omp parallel for schedule(dynamic)
#endif
	for (int y = 0; y < rows; ++y) {
		for (int x = 0; x < cols; ++x) {
			float val = 0;
			float weight_sum = 0;
			bool flag = false;
			for (int dy = -K; dy < K + 1; ++dy) {
				for (int dx = -K; dx < K + 1; ++dx) {
					const int u = x + dx;
					const int v = y + dy;
					if (0 <= u && u < cols && 0 <= v && v < rows)
					{
						//if (threshold < src(v, u)) {
						if (src(v, u) != offset_val) {
							const int d2 = dx * dx + dy * dy;
							const float w = std::exp(-beta * d2);
							val += w * (src(v, u) - offset_val);
							weight_sum += w;
							flag = true;
						}
					}
				}
			}
			dst(y, x) = flag ? ((val / weight_sum) + offset_val) : src(y, x);
		}
	}
}



void fx8_oc_pcl_data_subscribe_callback(const sensor_msgs::PointCloud2 &_cld)
{
	boost::lock_guard<boost::recursive_mutex> lock(mtx);

	pcl::fromROSMsg(_cld, *fx8_cld);


	//view
	//if (g_userdata != NULL)
	//{
	//	pcl::visualization::CloudViewer* pv = (pcl::visualization::CloudViewer*)(g_userdata);
	//	pv->showCloud(pcld);
	//}


	//PCL
	int pn = fx8_cld->points.size();
	for (int i = 0; i < pn; i++)
	{
		fx8_cld->points[i].x += 0 / 1000.0;				//offset of Fx8
		fx8_cld->points[i].y += 50 / 1000.0;
		fx8_cld->points[i].z -= 30 / 1000.0;
	}

	pcl::toROSMsg(*fx8_cld, sm_fx8cld);					
	sm_fx8cld.header.seq = -1;
	sm_fx8cld.header.frame_id = "map";




	//rgb/x/y/z_img
	std::vector<PXCPoint3DF32> cam_pos_list(pn);
	std::vector<PXCPointF32> color_pos_list(pn);

	for (int i = 0; i < pn; i++)
	{
		cam_pos_list[i].x = -1*(18 + fx8_cld->points[i].x * 1000.0);	//-1 means that realsense is left hand system
		cam_pos_list[i].y = fx8_cld->points[i].y * 1000.0;		//m -> mm		
		cam_pos_list[i].z = fx8_cld->points[i].z * 1000.0;

	}

	cv::Mat imgx, imgy, imgz;
	bool ret = rsg->conv_camera_to_color(cam_pos_list, imgx, imgy, imgz, false);
	if (ret == false)
	{
		return;
	}

	if (IMG_FLIP)
	{
		cv::flip(imgx, imgx, -1);
		cv::flip(imgy, imgy, -1);
		cv::flip(imgz, imgz, -1);
	}

	std_msgs::Header header; // empty header
	header.seq = -1; // user defined counter
	header.stamp = ros::Time::now(); // time


	sensor_msgs::Image imgx_msg;
	cv_bridge::CvImage imgx_bridge;
	imgx_bridge.header = header;
	imgx_bridge.encoding = sensor_msgs::image_encodings::MONO16;
	imgx_bridge.image = imgx;
	imgx_bridge.toImageMsg(imgx_msg); // from cv_bridge to sensor_msgs::Image

	sensor_msgs::Image imgy_msg;
	cv_bridge::CvImage imgy_bridge;
	imgy_bridge.header = header;
	imgy_bridge.encoding = sensor_msgs::image_encodings::MONO16;
	imgy_bridge.image = imgy;
	imgy_bridge.toImageMsg(imgy_msg); // from cv_bridge to sensor_msgs::Image

	sensor_msgs::Image imgz_msg;
	cv_bridge::CvImage imgz_bridge;
	imgz_bridge.header = header;
	imgz_bridge.encoding = sensor_msgs::image_encodings::MONO16;
	imgz_bridge.image = imgz;
	imgz_bridge.toImageMsg(imgz_msg); // from cv_bridge to sensor_msgs::Image


									
	if(PUB_PCL)
		fx8pcpub.publish(sm_fx8cld);

	fx8xpub.publish(imgx_msg);
	fx8ypub.publish(imgy_msg);
	fx8zpub.publish(imgz_msg);

	apc2016::XYZImage	xyzimg;
	xyzimg.x = imgx_msg;
	xyzimg.y = imgy_msg;
	xyzimg.z = imgz_msg;
	fx8xyzpub.publish(xyzimg);



	//visualization
	double vis_rate = 30;	//exaggeration rate for visualization

	int w = imgx.cols;
	int h = imgx.rows;

	cv::Mat imgx_vis(h, w, CV_16UC1);
	cv::Mat imgy_vis(h, w, CV_16UC1);
	cv::Mat imgz_vis(h, w, CV_16UC1);

	unsigned short* pxv = (unsigned short*)(imgx_vis.data);
	unsigned short* pyv = (unsigned short*)(imgy_vis.data);
	unsigned short* pzv = (unsigned short*)(imgz_vis.data);

	unsigned short* px = (unsigned short*)(imgx.data);
	unsigned short* py = (unsigned short*)(imgy.data);
	unsigned short* pz = (unsigned short*)(imgz.data);

	for (int y = 0; y < h; y++)
	{
		for (int x = 0; x < w; x++)
		{
			pxv[y*w + x] = ((int)(px[y*w + x]) - pcl::RealSenseGrabber::VALUE_OFFSET) * vis_rate + pcl::RealSenseGrabber::VALUE_OFFSET;
			pyv[y*w + x] = ((int)(py[y*w + x]) - pcl::RealSenseGrabber::VALUE_OFFSET) * vis_rate + pcl::RealSenseGrabber::VALUE_OFFSET;
			pzv[y*w + x] = ((int)(pz[y*w + x]) - pcl::RealSenseGrabber::VALUE_OFFSET) * vis_rate + pcl::RealSenseGrabber::VALUE_OFFSET;
		}
	}


	cv::imshow(std::string("fx8 x ") + WHERE, imgx_vis);
	cv::imshow(std::string("fx8 y ") + WHERE, imgy_vis);
	cv::imshow(std::string("fx8 z ") + WHERE, imgz_vis);



	//interpolation view
	int c = cv::waitKey(1);
	//printf("c:%d\n", c);
	if (c == 'h' && prev_key == -1)
	{
		hokan_window_flg = !hokan_window_flg;
	}
	prev_key = c;

	if (c == 'q')
		exit_flg = true;

	std::string hokan_x_wnm = std::string("fx8 x hokan ") + WHERE;
	std::string hokan_y_wnm = std::string("fx8 y hokan ") + WHERE;
	std::string hokan_z_wnm = std::string("fx8 z hokan ") + WHERE;
	if (hokan_window_flg == true)
	{
		cv::Mat_<unsigned short> x_ = imgx_vis;
		cv::Mat_<unsigned short> y_ = imgy_vis;
		cv::Mat_<unsigned short> z_ = imgz_vis;

		cv::Mat_<unsigned short> x_hokan(h, w);
		cv::Mat_<unsigned short> y_hokan(h, w);
		cv::Mat_<unsigned short> z_hokan(h, w);

		int hokan_area_size = 15;
		gaussian_interpolation(x_, x_hokan, hokan_area_size, (unsigned short)pcl::RealSenseGrabber::VALUE_OFFSET, 1.0f);
		gaussian_interpolation(y_, y_hokan, hokan_area_size, (unsigned short)pcl::RealSenseGrabber::VALUE_OFFSET, 1.0f);
		gaussian_interpolation(z_, z_hokan, hokan_area_size, (unsigned short)pcl::RealSenseGrabber::VALUE_OFFSET, 1.0f);

		cv::imshow(hokan_x_wnm, x_hokan);
		cv::imshow(hokan_y_wnm, y_hokan);
		cv::imshow(hokan_z_wnm, z_hokan);
	}
	else
	{
		cvDestroyWindow(hokan_x_wnm.c_str());
		cvDestroyWindow(hokan_y_wnm.c_str());
		cvDestroyWindow(hokan_z_wnm.c_str());
	}


}


void viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
{
	//viewer.setBackgroundColor(1.0, 0.5, 1.0);
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3);
}


char* conf_fname = "realsenes_node.conf";

int _tmain(int argc, _TCHAR * argv[])
{

	if (argc < 4)
	{
		printf("not enough arguments, ROS_MASTER_URI, where(left or right), device id(#1,#2,,) and imgs pub flip (1) optional\n");
		return 0;
	}


	strcpy(ROS_MASTER_STR, argv[1]);
	strcpy(WHERE, argv[2]);
	strcpy(DEVICEID, argv[3]);

	if (argc >= 5)
	{
		if (atoi(argv[4]) == 1)
			IMG_FLIP = true;
	}

	printf("<realsense_node conf>\n");
	printf("ROS_MASTER_URI=%s\n", ROS_MASTER_STR);
	printf("%s\n", WHERE);
	printf("devide id: %s\n", DEVICEID);

	//ROS
	char tmp[100];
	sprintf(tmp, "ROS_MASTER_URI=%s", ROS_MASTER_STR);
	putenv(tmp);
	fprintf(stdout, "ROS_MASTER_URI=%s\n", getenv("ROS_MASTER_URI"));


	printf("press h on window to show/hide fx8 hokan windows.\n");
	printf("press q on window to exit this node.\n");


	ros::init(argc, argv, std::string("realsense_node_") + WHERE);

	ros::NodeHandle nh;

	int queue_size = 2;

	ros::Publisher pcpub = nh.advertise<sensor_msgs::PointCloud2>(std::string("realsense_pcl_data_") + WHERE, queue_size);



	ros::Publisher rgbpub = nh.advertise<sensor_msgs::Image>(std::string("realsense_rgb_img_") + WHERE, queue_size);
	ros::Publisher xpub = nh.advertise<sensor_msgs::Image>(std::string("realsense_x_img_") + WHERE, queue_size);
	ros::Publisher ypub = nh.advertise<sensor_msgs::Image>(std::string("realsense_y_img_") + WHERE, queue_size);
	ros::Publisher zpub = nh.advertise<sensor_msgs::Image>(std::string("realsense_z_img_") + WHERE, queue_size);
	ros::Publisher xyzpub = nh.advertise<apc2016::XYZImage>(std::string("realsense_xyz_img_") + WHERE, queue_size);



	ros::Subscriber fx8sub = nh.subscribe(std::string("fx8_pcl_data_oc_") + WHERE, 10, fx8_oc_pcl_data_subscribe_callback);

	fx8pcpub = nh.advertise<sensor_msgs::PointCloud2>(std::string("fx8_pcl_data_") + WHERE, queue_size);
	fx8xpub = nh.advertise<sensor_msgs::Image>(std::string("fx8_x_img_") + WHERE, queue_size);
	fx8ypub = nh.advertise<sensor_msgs::Image>(std::string("fx8_y_img_") + WHERE, queue_size);
	fx8zpub = nh.advertise<sensor_msgs::Image>(std::string("fx8_z_img_") + WHERE, queue_size);
	fx8xyzpub = nh.advertise<apc2016::XYZImage>(std::string("fx8_xyz_img_") + WHERE, queue_size);



	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr seg_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
	sensor_msgs::PointCloud2	sm_cld;



	//pcl::visualization::PCLVisualizer* viewer = new pcl::visualization::PCLVisualizer("viewer");
	pcl::visualization::PCLVisualizer* viewer = NULL;

	if (viewer != NULL)
	{
		viewer->initCameraParameters();

		int v1 = 0;
		viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
		viewer->setBackgroundColor(0, 0, 0);

		viewer->addCoordinateSystem(0.05);

	}


	const double period_time_ms = 1000.0 / 5;
	double prev_time_ms1 = ros::Time::now().toSec() * 1000;

	int fcounter = 0;
	int z = 0;
	// Callback Function to be called when Updating Data
	boost::function<void(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> function =
		[&sm_cld, &pcpub, &nh, &prev_time_ms1, &period_time_ms, &seg_cloud, &fcounter](const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud) {


		double now_ms = ros::Time::now().toSec() * 1000;
		if ((now_ms - prev_time_ms1) < period_time_ms)
			return;

		prev_time_ms1 = now_ms;

		{
			boost::lock_guard<boost::recursive_mutex> lock(mtx);
			*seg_cloud = *cloud;
		}


		int pn = seg_cloud->points.size();

		for (int i = 0; i < pn; i++)
		{
			seg_cloud->points[i].x -= 23.0 / 1000;	//m
		}

		pcl::toROSMsg(*seg_cloud, sm_cld);

		sm_cld.header.seq = fcounter;
		sm_cld.header.frame_id = "map";
		fcounter++;

		if(PUB_PCL)
			pcpub.publish(sm_cld);


	};

	double prev_time_ms2 = ros::Time::now().toSec() * 1000;

	boost::function<void(const ImageXYZRGBA_sepa&)> function2 =
		[&rgbpub, &xpub, &ypub, &zpub, &xyzpub, &fcounter, &prev_time_ms2, &period_time_ms](const ImageXYZRGBA_sepa &sepa) {

		double now_ms = ros::Time::now().toSec() * 1000;
		if ((now_ms - prev_time_ms2) < period_time_ms)
			return;

		prev_time_ms2 = now_ms;


		cv::Mat img(sepa.h, sepa.w, CV_8UC3);

		for (int y = 0; y < sepa.h; y++)
		{
			for (int x = 0; x < sepa.w; x++)
			{
				img.data[(y*sepa.w + x) * 3 + 0] = sepa.rgba_data[(y*sepa.w + x) * 4 + 0];
				img.data[(y*sepa.w + x) * 3 + 1] = sepa.rgba_data[(y*sepa.w + x) * 4 + 1];
				img.data[(y*sepa.w + x) * 3 + 2] = sepa.rgba_data[(y*sepa.w + x) * 4 + 2];

			}
		}

		cv::Mat imgx(sepa.h, sepa.w, CV_16UC1);
		cv::Mat imgx_vis(sepa.h, sepa.w, CV_16UC1);
		cv::Mat imgy(sepa.h, sepa.w, CV_16UC1);
		cv::Mat imgy_vis(sepa.h, sepa.w, CV_16UC1);
		cv::Mat imgz(sepa.h, sepa.w, CV_16UC1);
		cv::Mat imgz_vis(sepa.h, sepa.w, CV_16UC1);
		unsigned short* px = (unsigned short*)(imgx.data);
		unsigned short* pxv = (unsigned short*)(imgx_vis.data);
		unsigned short* py = (unsigned short*)(imgy.data);
		unsigned short* pyv = (unsigned short*)(imgy_vis.data);
		unsigned short* pz = (unsigned short*)(imgz.data);
		unsigned short* pzv = (unsigned short*)(imgz_vis.data);

		const int VALUE_OFFSET = 30000;	//30000mm 30m
		for (int y = 0; y < sepa.h; y++)
		{
			for (int x = 0; x < sepa.w; x++)
			{
				px[y*sepa.w + x] = sepa.x_data[y*sepa.w + x] + VALUE_OFFSET;
				pxv[y*sepa.w + x] = sepa.x_data[y*sepa.w + x] * 30 + VALUE_OFFSET;
				py[y*sepa.w + x] = sepa.y_data[y*sepa.w + x] + VALUE_OFFSET;
				pyv[y*sepa.w + x] = sepa.y_data[y*sepa.w + x] * 30 + VALUE_OFFSET;
				pz[y*sepa.w + x] = sepa.z_data[y*sepa.w + x] + VALUE_OFFSET;
				pzv[y*sepa.w + x] = sepa.z_data[y*sepa.w + x] * 30 + VALUE_OFFSET;
			}
		}


		if (IMG_FLIP)
		{
			cv::flip(img, img, -1);

			cv::flip(imgx, imgx, -1);
			cv::flip(imgy, imgy, -1);
			cv::flip(imgz, imgz, -1);
			cv::flip(imgx_vis, imgx_vis, -1);
			cv::flip(imgy_vis, imgy_vis, -1);
			cv::flip(imgz_vis, imgz_vis, -1);
		}

		cv::imshow(std::string("img ") + WHERE, img);
		cv::imshow(std::string("x ") + WHERE, imgx_vis);
		cv::imshow(std::string("y ") + WHERE, imgy_vis);
		cv::imshow(std::string("z ") + WHERE, imgz_vis);
		int c = cv::waitKey(1);
		if (c == 'q')
			exit_flg = true;


		std_msgs::Header header; // empty header
		header.seq = fcounter; // user defined counter
		header.stamp = ros::Time::now(); // time

		sensor_msgs::Image img_msg;
		cv_bridge::CvImage img_bridge;
		img_bridge.header = header;
		img_bridge.encoding = sensor_msgs::image_encodings::BGR8;// RGB8;
		img_bridge.image = img;
		img_bridge.toImageMsg(img_msg); // from cv_bridge to sensor_msgs::Image

		sensor_msgs::Image imgx_msg;
		cv_bridge::CvImage imgx_bridge;
		imgx_bridge.header = header;
		imgx_bridge.encoding = sensor_msgs::image_encodings::MONO16;
		imgx_bridge.image = imgx;
		imgx_bridge.toImageMsg(imgx_msg); // from cv_bridge to sensor_msgs::Image

		sensor_msgs::Image imgy_msg;
		cv_bridge::CvImage imgy_bridge;
		imgy_bridge.header = header;
		imgy_bridge.encoding = sensor_msgs::image_encodings::MONO16;
		imgy_bridge.image = imgy;
		imgy_bridge.toImageMsg(imgy_msg); // from cv_bridge to sensor_msgs::Image

		sensor_msgs::Image imgz_msg;
		cv_bridge::CvImage imgz_bridge;
		imgz_bridge.header = header;
		imgz_bridge.encoding = sensor_msgs::image_encodings::MONO16;
		imgz_bridge.image = imgz;
		imgz_bridge.toImageMsg(imgz_msg); // from cv_bridge to sensor_msgs::Image

		rgbpub.publish(img_msg); // ros::Publisher pub_img = node.advertise<sensor_msgs
		xpub.publish(imgx_msg);
		ypub.publish(imgy_msg);
		zpub.publish(imgz_msg);


		apc2016::XYZImage	xyzimg;
		xyzimg.x = imgx_msg;
		xyzimg.y = imgy_msg;
		xyzimg.z = imgz_msg;
		xyzpub.publish(xyzimg);

	};


	// Create Kinect2Grabber
	//boost::shared_ptr<pcl::Grabber> grabber = boost::make_shared<pcl::Kinect2Grabber>();
	rsg = new pcl::RealSenseGrabber(DEVICEID);
	boost::shared_ptr<pcl::RealSenseGrabber> rsgrabber(rsg);
	boost::shared_ptr<pcl::Grabber> grabber = rsgrabber;


	// Regist Callback Function
	//bool pv = grabber->providesCallback();
	grabber->registerCallback(function);
	grabber->registerCallback(function2);


	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr mg(new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr fxmg(new pcl::PointCloud<pcl::PointXYZ>);


	//pcl::visualization::CloudViewer	cv("hoge");


	// Start Retrieve Data
	grabber->start();

	ros::Rate loop_rate(100);	//100hz
	while (ros::ok())
	{
		// Input Key ( Exit ESC key )
		//if (GetKeyState(VK_ESCAPE) < 0) {
		//	break;
		//}

		if (exit_flg == true)
			break;


		ros::spinOnce();



							
		if (viewer != NULL)
		{
			boost::lock_guard<boost::recursive_mutex> lock(mtx);

			viewer->removeAllPointClouds();

			viewer->addPointCloud(seg_cloud, "cloud");
			viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");

			viewer->addPointCloud(fx8_cld, "fx8");
			viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "fx8");
			viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 1.0, "fx8");

			viewer->spinOnce();
		}

		//
		{
			{
				boost::lock_guard<boost::recursive_mutex> lock(mtx);
				*mg = *seg_cloud;
				*fxmg = *fx8_cld;
			}

			int on = mg->points.size();
			mg->points.resize(mg->points.size() + fxmg->points.size());
			for (int i = 0; i < fxmg->points.size(); i++)
			{
				mg->points[on + i].x = fxmg->points[i].x;
				mg->points[on + i].y = fxmg->points[i].y;
				mg->points[on + i].z = fxmg->points[i].z;

				mg->points[on + i].r = 255;
				mg->points[on + i].g = 0;
				mg->points[on + i].b = 0;
			}

			//cv.runOnVisualizationThreadOnce(viewerOneOff);
			//cv.showCloud(mg);
		}
		loop_rate.sleep();
	}

	// Stop Retrieve Data
	grabber->stop();

	return 0;
}
