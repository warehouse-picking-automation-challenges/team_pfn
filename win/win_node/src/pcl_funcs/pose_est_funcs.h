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


#include <vector>
#include <string>
#include <sstream>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/pcl_config.h>

#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/keypoints/harris_6d.h>
#include <pcl/keypoints/uniform_sampling.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/pfh.h>
#include <pcl/features/pfhrgb.h>
#include <pcl/features/3dsc.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/ppfrgb.h>
#include <pcl/features/pfhrgb.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/common/transforms.h>
#include <pcl/surface/grid_projection.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/marching_cubes_hoppe.h>


#include "../pcl_cfg.h"


struct POSE_EST_PARAMS
{
	POSE_EST_PARAMS()
	{
		leaf = 0.0025;
		norm_radius = 0.01;
		feature_radius = 0.025;

		MaximumIterations = 50000;
		NumberOfSamples = 3;
		SimilarityThreshold = 0.6;
		CorrespondenceRandomness = 5;
		MaxCorrespondenceDistance = 3.0 * leaf;
		InlierFraction = 0.15;
	}
	double leaf;
	double norm_radius;
	double feature_radius;

	int MaximumIterations;
	int NumberOfSamples;
	double SimilarityThreshold;
	double MaxCorrespondenceDistance;
	int CorrespondenceRandomness;
	double InlierFraction;

	bool read_params(const char* fname);
	void print_params();
};

bool pose_est_(
	pcl::PointCloud<pcl::PointXYZRGB>& model_,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr model_aligned,
	double& cx, double& cy, double& cz, double &ax, double& ay, double& az,
	const POSE_EST_PARAMS& params);

bool pose_est(
	pcl::PointCloud<pcl::PointXYZRGB>& model_,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr model_aligned,
	double& cx, double& cy, double& cz, double &ax, double& ay, double& az);

bool pose_est2(
	pcl::PointCloud<pcl::PointXYZRGB>& model_,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr model_aligned,
	double& cx, double& cy, double& cz, double &ax, double& ay, double& az);
