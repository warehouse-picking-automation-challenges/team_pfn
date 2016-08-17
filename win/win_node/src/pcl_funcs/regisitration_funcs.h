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
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/common/transforms.h>
#include <pcl/surface/grid_projection.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/marching_cubes_hoppe.h>



#include "../src/pcl_cfg.h"



void segmentation(
	pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input_cl,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmented);

void detectKeypoints(
	boost::shared_ptr<pcl::Keypoint<pcl::PointXYZRGB, pcl::PointXYZI> > keypoint_detector_,
	pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input,
	pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints);

void extractDescriptors(
	pcl::Feature<pcl::PointXYZRGB, pcl::SHOT1344>::Ptr feature_extractor_,
	pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input,
	pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints,
	pcl::PointCloud<pcl::SHOT1344>::Ptr features);

void findCorrespondences(
	pcl::PointCloud<pcl::SHOT1344>::Ptr model_ft,
	pcl::PointCloud<pcl::SHOT1344>::Ptr target_ft,
	std::vector<int>& correspondences);

void filterCorrespondences(
	pcl::PointCloud<pcl::PointXYZI>::Ptr model_keypoints_,
	pcl::PointCloud<pcl::PointXYZI>::Ptr target_keypoints_,
	std::vector<int>& correspondences_model2target_,
	std::vector<int>& correspondences_target2model_,
	pcl::CorrespondencesPtr filtered_correspondences);

void determineInitialTransformation(
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr model_segmented_,
	pcl::CorrespondencesPtr correspondences_,
	pcl::PointCloud<pcl::PointXYZI>::Ptr model_keypoints_,
	pcl::PointCloud<pcl::PointXYZI>::Ptr target_keypoints_,
	Eigen::Matrix4f& initial_transformation_matrix_,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr model_cl_initial_transformed_);
void determineInitialTransformation(
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr model_segmented_,
	pcl::CorrespondencesPtr correspondences_,
	pcl::PointCloud<pcl::PointXYZI>::Ptr model_keypoints_,
	pcl::PointCloud<pcl::PointXYZI>::Ptr target_keypoints_,
	Eigen::Matrix4f& initial_transformation_matrix_,
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr model_cl_initial_transformed_);

void determineInitialTransformation(
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr model_segmented_,
	pcl::CorrespondencesPtr correspondences_,
	pcl::PointCloud<pcl::PointXYZI>::Ptr model_keypoints_,
	pcl::PointCloud<pcl::PointXYZI>::Ptr target_keypoints_,
	Eigen::Matrix4f& initial_transformation_matrix_);

void determineFinalTransformation(
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr model_cl_init_transformed_,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_cl_,
	Eigen::Matrix4f& transformation_matrix_,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr model_cl_transformed_);

void reconstructSurface(
	boost::shared_ptr<pcl::PCLSurfaceBase<pcl::PointXYZRGBNormal> > surface_reconstructor_,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr model_transformed_,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_segmented_,
	pcl::PolygonMesh& surface_);


