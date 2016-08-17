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

// This file incorporates code from https://github.com/PointCloudLibrary/pcl/tree/456bf44205ac7ed09b2754e2062d08a1a7fbed53/doc/tutorials/content/sources/iccv2011/src


#include "regisitration_funcs.h"


void segmentation(
	pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input_cl,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmented)
{
	cout << "segmentation..." << std::flush;
	// fit plane and keep points above that plane
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZRGB> seg;
	
	// Optional
	seg.setOptimizeCoefficients(true);
	
	// Mandatory
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(0.02);

	seg.setInputCloud(input_cl);
	seg.segment(*inliers, *coefficients);

	pcl::ExtractIndices<pcl::PointXYZRGB> extract;
	extract.setInputCloud(input_cl);
	extract.setIndices(inliers);
	extract.setNegative(true);

	extract.filter(*segmented);
	std::vector<int> indices;
	pcl::removeNaNFromPointCloud(*segmented, *segmented, indices);
	cout << "OK" << endl;

	cout << "clustering..." << std::flush;
	// euclidean clustering
	typename pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
	tree->setInputCloud(segmented);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> clustering;
	clustering.setClusterTolerance(0.02); // 2cm
	clustering.setMinClusterSize(1000);
	clustering.setMaxClusterSize(250000);
	clustering.setSearchMethod(tree);
	clustering.setInputCloud(segmented);
	clustering.extract(cluster_indices);

	if (cluster_indices.size() > 0)//use largest cluster
	{
		cout << cluster_indices.size() << " clusters found";
		if (cluster_indices.size() > 1)
			cout << " Using largest one...";
		cout << endl;
		typename pcl::IndicesPtr indices(new std::vector<int>);
		*indices = cluster_indices[0].indices;
		extract.setInputCloud(segmented);
		extract.setIndices(indices);
		extract.setNegative(false);

		extract.filter(*segmented);
	}
}

void detectKeypoints(
	boost::shared_ptr<pcl::Keypoint<pcl::PointXYZRGB, pcl::PointXYZI> > keypoint_detector_,
	pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input,
	pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints)
{
	cout << "keypoint detection..." << std::flush;
	keypoint_detector_->setInputCloud(input);
	keypoint_detector_->setSearchSurface(input);
	keypoint_detector_->compute(*keypoints);
	cout << "OK. keypoints found: " << keypoints->points.size() << endl;
}


void extractDescriptors(
	pcl::Feature<pcl::PointXYZRGB, pcl::SHOT1344>::Ptr feature_extractor_,
	pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input,
	pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints,
	pcl::PointCloud<pcl::SHOT1344>::Ptr features)
{
	typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr kpts(new pcl::PointCloud<pcl::PointXYZRGB>);
	kpts->points.resize(keypoints->points.size());

	pcl::copyPointCloud(*keypoints, *kpts);

	typename pcl::FeatureFromNormals<pcl::PointXYZRGB, pcl::Normal, pcl::SHOT1344>::Ptr feature_from_normals = boost::dynamic_pointer_cast<pcl::FeatureFromNormals<pcl::PointXYZRGB, pcl::Normal, pcl::SHOT1344> > (feature_extractor_);

	feature_extractor_->setSearchSurface(input);
	feature_extractor_->setInputCloud(kpts);

	if (feature_from_normals)
		//if (boost::dynamic_pointer_cast<typename pcl::FeatureFromNormals<pcl::PointXYZRGB, pcl::Normal, pcl::SHOT1344> > (feature_extractor_))
	{
		cout << "normal estimation..." << std::flush;
		typename pcl::PointCloud<pcl::Normal>::Ptr normals(new  pcl::PointCloud<pcl::Normal>);
		pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimation;
		normal_estimation.setSearchMethod(pcl::search::Search<pcl::PointXYZRGB>::Ptr(new pcl::search::KdTree<pcl::PointXYZRGB>));
		normal_estimation.setRadiusSearch(0.01);
		normal_estimation.setInputCloud(input);
		normal_estimation.compute(*normals);
		feature_from_normals->setInputNormals(normals);
		cout << "OK" << endl;
	}

	cout << "descriptor extraction..." << std::flush;
	feature_extractor_->compute(*features);
	cout << "OK" << endl;
}


void findCorrespondences(
	pcl::PointCloud<pcl::SHOT1344>::Ptr model_ft,
	pcl::PointCloud<pcl::SHOT1344>::Ptr target_ft,
	std::vector<int>& correspondences)
{
	cout << "correspondence assignment..." << std::flush;
	correspondences.resize(model_ft->size());

	// Use a KdTree to search for the nearest matches in feature space
	pcl::KdTreeFLANN<pcl::SHOT1344> descriptor_kdtree;
	descriptor_kdtree.setInputCloud(target_ft);

	// Find the index of the best match for each keypoint, and store it in "correspondences_out"
	const int k = 1;
	std::vector<int> k_indices(k);
	std::vector<float> k_squared_distances(k);
	for (size_t i = 0; i < model_ft->size(); ++i)
	{
		descriptor_kdtree.nearestKSearch(*model_ft, i, k, k_indices, k_squared_distances);
		correspondences[i] = k_indices[0];
	}
	cout << "OK" << endl;
}


void filterCorrespondences(
	pcl::PointCloud<pcl::PointXYZI>::Ptr model_keypoints_,
	pcl::PointCloud<pcl::PointXYZI>::Ptr target_keypoints_,
	std::vector<int>& correspondences_model2target_,
	std::vector<int>& correspondences_target2model_,
	pcl::CorrespondencesPtr filtered_correspondences)
{
	cout << "correspondence rejection..." << std::flush;
	std::vector<std::pair<unsigned, unsigned> > correspondences;
	for (unsigned cIdx = 0; cIdx < correspondences_model2target_.size(); ++cIdx)
		if (correspondences_target2model_[correspondences_model2target_[cIdx]] == cIdx)
			correspondences.push_back(std::make_pair(cIdx, correspondences_model2target_[cIdx]));

	filtered_correspondences->resize(correspondences.size());
	for (unsigned cIdx = 0; cIdx < correspondences.size(); ++cIdx)
	{
		(*filtered_correspondences)[cIdx].index_query = correspondences[cIdx].first;
		(*filtered_correspondences)[cIdx].index_match = correspondences[cIdx].second;
	}

	pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZI> rejector;
	rejector.setInputCloud(model_keypoints_);
	rejector.setTargetCloud(target_keypoints_);
	rejector.setInputCorrespondences(filtered_correspondences);
	rejector.getCorrespondences(*filtered_correspondences);
	cout << "OK" << endl;
}

void determineInitialTransformation(
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr model_segmented_,
	pcl::CorrespondencesPtr correspondences_,
	pcl::PointCloud<pcl::PointXYZI>::Ptr model_keypoints_,
	pcl::PointCloud<pcl::PointXYZI>::Ptr target_keypoints_,
	Eigen::Matrix4f& initial_transformation_matrix_,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr model_cl_initial_transformed_)
{
	cout << "initial alignment..." << std::flush;
	pcl::registration::TransformationEstimation<pcl::PointXYZI, pcl::PointXYZI>::Ptr transformation_estimation(new pcl::registration::TransformationEstimationSVD<pcl::PointXYZI, pcl::PointXYZI>);

	transformation_estimation->estimateRigidTransformation(*model_keypoints_, *target_keypoints_, *correspondences_, initial_transformation_matrix_);

	pcl::transformPointCloud(*model_segmented_, *model_cl_initial_transformed_, initial_transformation_matrix_);
	cout << "OK" << endl;
}

void determineInitialTransformation(
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr model_segmented_,
	pcl::CorrespondencesPtr correspondences_,
	pcl::PointCloud<pcl::PointXYZI>::Ptr model_keypoints_,
	pcl::PointCloud<pcl::PointXYZI>::Ptr target_keypoints_,
	Eigen::Matrix4f& initial_transformation_matrix_,
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr model_cl_initial_transformed_)
{
	cout << "initial alignment..." << std::flush;
	pcl::registration::TransformationEstimation<pcl::PointXYZI, pcl::PointXYZI>::Ptr transformation_estimation(new pcl::registration::TransformationEstimationSVD<pcl::PointXYZI, pcl::PointXYZI>);

	transformation_estimation->estimateRigidTransformation(*model_keypoints_, *target_keypoints_, *correspondences_, initial_transformation_matrix_);

	pcl::transformPointCloud(*model_segmented_, *model_cl_initial_transformed_, initial_transformation_matrix_);
	cout << "OK" << endl;
}


void determineInitialTransformation(
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr model_segmented_,
	pcl::CorrespondencesPtr correspondences_,
	pcl::PointCloud<pcl::PointXYZI>::Ptr model_keypoints_,
	pcl::PointCloud<pcl::PointXYZI>::Ptr target_keypoints_,
	Eigen::Matrix4f& initial_transformation_matrix_)
{
	cout << "initial alignment..." << std::flush;
	pcl::registration::TransformationEstimation<pcl::PointXYZI, pcl::PointXYZI>::Ptr transformation_estimation(new pcl::registration::TransformationEstimationSVD<pcl::PointXYZI, pcl::PointXYZI>);

	transformation_estimation->estimateRigidTransformation(*model_keypoints_, *target_keypoints_, *correspondences_, initial_transformation_matrix_);

}



void determineFinalTransformation(
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr model_cl_init_transformed_,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_cl_,
	Eigen::Matrix4f& transformation_matrix_,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr model_cl_transformed_)
{
	cout << "final registration..." << std::flush;
	pcl::Registration<pcl::PointXYZRGB, pcl::PointXYZRGB>::Ptr registration(new pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB>);
	registration->setInputCloud(model_cl_init_transformed_);
	registration->setInputTarget(target_cl_);

	//registration->setMaxCorrespondenceDistance(0.05);
	//registration->setRANSACOutlierRejectionThreshold(0.05);
	registration->setMaxCorrespondenceDistance(0.1);			//èàóùëŒè€ãóó£
	registration->setRANSACOutlierRejectionThreshold(0.1);		//ëŒè€äOÇ∆Ç∑ÇÈãóó£
	
	//registration->setTransformationEpsilon(0.000001);
	registration->setTransformationEpsilon(0.00005);
	
	registration->setMaximumIterations(1000);
	//registration->setMaximumIterations(100);


	registration->align(*model_cl_transformed_);
	transformation_matrix_ = registration->getFinalTransformation();
	cout << "OK" << endl;
}

void reconstructSurface(
	boost::shared_ptr<pcl::PCLSurfaceBase<pcl::PointXYZRGBNormal> > surface_reconstructor_,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr model_transformed_,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_segmented_,
	pcl::PolygonMesh& surface_)
{
	cout << "hoge";
	cout << "surface reconstruction..." << std::flush;
	// merge the transformed and the target point cloud
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr merged(new pcl::PointCloud<pcl::PointXYZRGB>);
	*merged = *model_transformed_;
	*merged += *target_segmented_;

	// apply grid filtering to reduce amount of points as well as to make them uniform distributed
	pcl::VoxelGrid<pcl::PointXYZRGB> voxel_grid;
	voxel_grid.setInputCloud(merged);
	voxel_grid.setLeafSize(0.002, 0.002, 0.002);
	voxel_grid.setDownsampleAllData(true);
	voxel_grid.filter(*merged);

	//èëÇ´èoÇµ
	int r = pcl::io::savePCDFile("finalRes.pcd", *merged);
	//printf("res:%d\n", r);
	cout << "res:" << r;

	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr vertices(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::copyPointCloud(*merged, *vertices);

	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::PointXYZRGBNormal> normal_estimation;
	normal_estimation.setSearchMethod(pcl::search::Search<pcl::PointXYZRGB>::Ptr(new pcl::search::KdTree<pcl::PointXYZRGB>));
	normal_estimation.setRadiusSearch(0.01);
	normal_estimation.setInputCloud(merged);
	normal_estimation.compute(*vertices);

	pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
	tree->setInputCloud(vertices);

	surface_reconstructor_->setSearchMethod(tree);
	surface_reconstructor_->setInputCloud(vertices);
	surface_reconstructor_->reconstruct(surface_);
	cout << "OK" << endl;

	cout << "res:" << r;
}