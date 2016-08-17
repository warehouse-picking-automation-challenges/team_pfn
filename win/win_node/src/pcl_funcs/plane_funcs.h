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

#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/segmentation/sac_segmentation.h>  
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>

#include "../pcl_cfg.h"

bool detect_largest_plane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cld,
                          double det_plane_dist,	//dist from plane
						  pcl::PointIndices::Ptr out_plane_index,
	                      pcl::ModelCoefficients::Ptr out_coeffi);

void extract_or_delete_by_index(pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cld,
	                         pcl::PointIndices::Ptr in_index,
	                         bool proc_is_delete,
	                         pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cld);


void extract_biggest_object_on_plane(
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cld,
	double det_plane_dist,	//dist from plane
	int min_obj_pt_num,
	int max_obj_pt_num,
	double obj_clst_dist,	//point dist as one obj
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cld);


void extract_objects(
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cld,
	int min_obj_pt_num,
	int max_obj_pt_num,
	double obj_clst_dist,
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& out_cld_list);

void extract_bigger_objects(
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cld,
	int min_obj_pt_num,
	int max_obj_pt_num,
	double obj_clst_dist,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cld);