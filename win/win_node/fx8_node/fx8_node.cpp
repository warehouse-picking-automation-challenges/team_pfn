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


//need comment out for struct FPSCallback : public vtkCommand in pcl/visualization/pcl_visualizer.h


//use cli and cpp
#ifdef __cplusplus_cli
#define generic __identifier(generic)
#endif

#include "../src/opencv_include.h"

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


//#include <pcl/visualization/cloud_viewer.h>

#include <pcl/segmentation/sac_segmentation.h>  
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>

#include "../src/pcl_cfg.h"


//use cli and cpp
#ifdef __cplusplus_cli
#undef generic
#endif



using namespace System;


ros::Publisher g_pcpub;
void* g_viewer;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;


public ref class Sensor
{
	private: System::ComponentModel::IContainer^  components;

	private: fx8::fx8userlib^  fx8userlib1;
	

			 System::String ^ ipadder;

public:
	

	Void init(System::String ^ _ipadder)
	{
		this->ipadder = _ipadder;

		this->components = (gcnew System::ComponentModel::Container());
		this->fx8userlib1 = (gcnew fx8::fx8userlib(this->components));

		this->fx8userlib1->ConnectTimeout = 10000;
		this->fx8userlib1->ReceiveTimeout = 5000;
		this->fx8userlib1->SendTimeout = 3000;
		this->fx8userlib1->SetSensorTimeEnabled = true;
		this->fx8userlib1->SetSensorTimeInterval = 3600000;
		this->fx8userlib1->ReceiveErrorInfo += gcnew fx8::fx8userlib::ReceiveErrorInfoEventHandler(this, &Sensor::fx8userlib1_ReceiveErrorInfo);
		this->fx8userlib1->ReceiveRangeData += gcnew fx8::fx8userlib::ReceiveRangeDataEventHandler(this, &Sensor::fx8userlib1_ReceiveRangeData);
		// 

	}

	Void Start()
	{
		// connect to sensor
		int iRet = fx8userlib1->Connect(this->ipadder, 50000);
		if (iRet != 0)
		{
			Console::WriteLine(L"Failed sensor connection.");
			return;
		}

		//set sensor to stop mode certainly
		iRet = fx8userlib1->StopRanging();
		if (iRet != 0)
		{
			fx8userlib1->Disconnect();
			Console::WriteLine(L"Ranging stop failed.");
			return;
		}

		//set sensor property
		iRet = fx8userlib1->SetSensorProperty();
		if (iRet != 0)
		{
			fx8userlib1->Disconnect();
			Console::WriteLine(L"Failed to set sensor property.");
			return;
		}

		//start
		iRet = fx8userlib1->StartRanging();
		if (iRet != 0)
		{
			fx8userlib1->Disconnect();
			Console::WriteLine(L"Ranging start failed.");
			return;
		}

	}

	Void fx8userlib1_ReceiveRangeData(array<Byte> ^RangeData)
	{

		Byte surf = RangeData[6];		// surface id
		fx8::fx8userlib::SensorXyDataElem ^elem = nullptr;
		UInt16 bright = 0;
		UInt16 range = 0;
		double calcRange = 0.0;
		UInt16 index = 0;

		static const double RangePerDigit = 4.0;		// absolute distance per 1digit (mm)


														//check type of received data
		if ((RangeData[4] != 0x01) || (RangeData[5] != 0x71))
		{
			return;
		}


		int pnum = fx8userlib1->XyData[surf]->Elem->Length;


		cloud->width = pnum;
		cloud->height = 1;
		cloud->is_dense = false;
		cloud->points.resize(pnum);


		for (int i = 0; i < pnum; i++)
		{
			elem = fx8userlib1->XyData[surf]->Elem[i];

			// distance
			calcRange = 0.0;
			range = (RangeData[i * 3 + 10] & 0x0F) << 8;
			range = range + RangeData[i * 3 + 11];

			calcRange = range * RangePerDigit;

			// convert distance value to xyz coordinates
			cloud->points[i].x = -1*-(float)calcRange * elem->By / 1000.0;	
			cloud->points[i].y = -(float)calcRange * elem->Bx / 1000.0;

			//x,y coordiantes are opposit direction
			//and, realsense places face down,
			//then put minus to x and y to become this direction as total camera system's coordinates directions

			//FX8 is left hand system (x coordinates is right direction), to convert it to right hand system , put minus to x

			//@©x  ªy  z(from the front to the back)
	
			cloud->points[i].z = (float)calcRange * elem->Bz / 1000.0;

		}


		
		sensor_msgs::PointCloud2	sm_cld;



		pcl::toROSMsg(*cloud, sm_cld);

		sm_cld.header.seq = -1;
		sm_cld.header.frame_id = "map";

		g_pcpub.publish(sm_cld);


	}

	Void fx8userlib1_ReceiveErrorInfo(array<Byte> ^ErrorData)
	{
		String ^str;

		//check type of received data
		if ((ErrorData[4] != 0x01) || (ErrorData[5] != 0x91))
		{
			return;
		}

		str = String::Format("Error:0x{0:X2}{1:X2}", ErrorData[6], ErrorData[7]);
		for (int i = 0; i < 6; i++)
		{
			str += String::Format(", Code({0}):0x{1:X2}", i, ErrorData[i + 8]);
		}
		Console::WriteLine(str);
	}
};


static std::string systemStringToStdString(System::String ^ s) {
	using namespace System::Runtime::InteropServices;

	IntPtr hString = Marshal::StringToHGlobalAnsi(s);
	//if (hString == 0) return "";

	std::string rtnSts = (const char *)hString.ToPointer();
	Marshal::FreeHGlobal(hString);
	return rtnSts;
}

//#pragma unmanaged
int main(array<System::String ^> ^args)
{
	if (args->Length < 3)
	{
		printf("not enough arguments, ROS_MASTER_URI, where(left or right) and ip address of fx8\n");
		return 0;
	}

	char ROS_MASTER_STR[100] = "\0";
	char WHERE[100] = "\0";
	System::String ^ IPADDER;

	strcpy(ROS_MASTER_STR, systemStringToStdString(args[0]).c_str());
	strcpy(WHERE, systemStringToStdString(args[1]).c_str());
	IPADDER = args[2];

	printf("<fx8_node conf>\n");
	printf("ROS_MASTER_URI=%s\n", ROS_MASTER_STR);
	printf("%s\n", WHERE);


	//ROS
	//putenv("ROS_MASTER_URI=http://192.168.0.1:11311");
	char tmp[100];
	sprintf(tmp, "ROS_MASTER_URI=%s", ROS_MASTER_STR);
	putenv(tmp);
	fprintf(stdout, "ROS_MASTER_URI=%s\n", getenv("ROS_MASTER_URI"));




	int argc = 0;
	char** argv = NULL;
	ros::init(argc, argv, std::string("fx8_node_") + WHERE);

	ros::NodeHandle nh;

	g_pcpub = nh.advertise<sensor_msgs::PointCloud2>(std::string("fx8_pcl_data_oc_") + WHERE, 10);


	pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud(new pcl::PointCloud<pcl::PointXYZ>);

	cloud = _cloud;


	//viewer
	g_viewer = NULL;
	//pcl::visualization::CloudViewer viewer("Test PCL Node Viewer");
	//g_viewer = &viewer;
	



	Sensor	sr;

	sr.init(IPADDER);

	sr.Start();


	ros::Rate loop_rate(100);	//100hz
	while (ros::ok())
	{
		//if (GetKeyState(VK_ESCAPE) < 0) {
		//	break;
		//}

		ros::spinOnce();

		loop_rate.sleep();
	}


    return 0;
}
