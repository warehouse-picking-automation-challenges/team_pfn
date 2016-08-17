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

#include "filter_funcs.h"


void statistic_outlier_remove_filter(
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cld,
	int calc_tgt_pt_num,	//calc target pt num
	double stddev_th,	//standard deviation threshold
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cld)
{
	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;

	sor.setInputCloud(in_cld);
	sor.setMeanK(calc_tgt_pt_num);
	sor.setStddevMulThresh(stddev_th);
	sor.setNegative(false);
	
	sor.filter(*out_cld);
}

void radius_outlier_remove_filter(
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cld,
	double radius,
	int min_pt_num,	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cld)
{
	pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> outrem;
	// build the filter
	outrem.setInputCloud(in_cld);
	outrem.setRadiusSearch(radius);
	outrem.setMinNeighborsInRadius(min_pt_num);

	outrem.filter(*out_cld);
}


