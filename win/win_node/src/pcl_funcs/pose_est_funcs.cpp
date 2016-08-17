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

#include "pose_est_funcs.h"
#include "filter_funcs.h"
#include "regisitration_funcs.h"

#include <pcl/visualization/cloud_viewer.h>

typedef pcl::PointXYZRGBNormal PointNT;
typedef pcl::PointCloud<PointNT> PointCloudT;
typedef pcl::FPFHSignature33 FeatureT;

typedef pcl::FPFHEstimationOMP<PointNT, PointNT, FeatureT> FeatureEstimationT;

typedef pcl::PointCloud<FeatureT> FeatureCloudT;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointNT> ColorHandlerT;

typedef pcl::visualization::PointCloudColorHandlerRGBField<PointNT> ColorHandlerRGB;


void computeAnglesFromMatrix(
	Eigen::Matrix4f& R,
	double & angle_x,
	double & angle_y,
	double & angle_z
) {

	double threshold = 0.001;

	if (abs(R(2, 1) - 1.0) < threshold) { // R(2,1) = sin(x) = 1の時
		angle_x = 3.14159265 / 2;
		angle_y = 0;
		angle_z = atan2(R(1, 0), R(0, 0));
	}
	else if (abs(R(2, 1) + 1.0) < threshold) { // R(2,1) = sin(x) = -1の時
		angle_x = -3.14159265 / 2;
		angle_y = 0;
		angle_z = atan2(R(1, 0), R(0, 0));
	}
	else {
		angle_x = asin(R(2, 1));
		angle_y = atan2(-R(2, 0), R(2, 2));
		angle_z = atan2(-R(0, 1), R(1, 1));
	}
}

void print_mat(Eigen::Matrix4f& transformation)
{
	pcl::console::print_info("    | %6.3f %6.3f %6.3f | \n", transformation(0, 0), transformation(0, 1), transformation(0, 2));
	pcl::console::print_info("R = | %6.3f %6.3f %6.3f | \n", transformation(1, 0), transformation(1, 1), transformation(1, 2));
	pcl::console::print_info("    | %6.3f %6.3f %6.3f | \n", transformation(2, 0), transformation(2, 1), transformation(2, 2));
	pcl::console::print_info("\n");
	pcl::console::print_info("t = < %0.3f, %0.3f, %0.3f >\n", transformation(0, 3), transformation(1, 3), transformation(2, 3));
}

void add_cld(PointCloudT::Ptr a, PointCloudT::Ptr b)
{
	a->points.insert(a->points.end(), b->points.begin(), b->points.end());
	a->is_dense = false;
}


bool POSE_EST_PARAMS::read_params(const char* fname)
{
	FILE* fp = fopen(fname, "r");

	int rn;
	rn = fscanf(fp, "leaf:%lf\n", &leaf);	if (rn != 1) { printf("read miss leaf!\n"); fclose(fp); return false; }
	rn = fscanf(fp, "norm_radius:%lf\n", &norm_radius);	if (rn != 1) { printf("read miss norm_radius!\n"); fclose(fp); return false; }
	rn = fscanf(fp, "feature_radius:%lf\n", &feature_radius);	if (rn != 1) { printf("read miss feature_radius!\n"); fclose(fp); return false; }
	rn = fscanf(fp, "MaximumIterations:%d\n", &MaximumIterations);	if (rn != 1) { printf("read miss MaximumIterations!\n"); fclose(fp); return false; }
	rn = fscanf(fp, "NumberOfSamples:%d\n", &NumberOfSamples);	if (rn != 1) { printf("read miss NumberOfSamples!\n"); fclose(fp); return false; }
	rn = fscanf(fp, "SimilarityThreshold:%lf\n", &SimilarityThreshold);	if (rn != 1) { printf("read miss SimilarityThreshold!\n"); fclose(fp); return false; }
	rn = fscanf(fp, "CorrespondenceRandomness:%d\n", &CorrespondenceRandomness);	if (rn != 1) { printf("read miss CorrespondenceRandomness!\n"); fclose(fp); return false; }
	rn = fscanf(fp, "MaxCorrespondenceDistance:%lf\n", &MaxCorrespondenceDistance);	if (rn != 1) { printf("read miss MaxCorrespondenceDistance!\n"); fclose(fp); return false; }
	rn = fscanf(fp, "InlierFraction:%lf", &InlierFraction);	if (rn != 1) { printf("read miss InlierFraction!\n"); fclose(fp); return false; }
	 
	return true;
}

void POSE_EST_PARAMS::print_params()
{
	printf("<pose est params>\n");
	printf( "leaf:%lf\n", leaf);
	printf( "norm_radius:%lf\n", norm_radius);
	printf( "feature_radius:%lf\n", feature_radius);
	printf( "MaximumIterations:%d\n", MaximumIterations);
	printf( "NumberOfSamples:%d\n", NumberOfSamples);
	printf( "SimilarityThreshold:%lf\n", SimilarityThreshold);
	printf( "CorrespondenceRandomness:%d\n", CorrespondenceRandomness);
	printf("MaxCorrespondenceDistance:%lf\n", MaxCorrespondenceDistance);
	printf( "InlierFraction:%lf\n", InlierFraction);
	printf("\n");

}

bool pose_est_(
	pcl::PointCloud<pcl::PointXYZRGB>& model_,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr model_aligned,
	double& cx, double& cy, double& cz, double &ax, double& ay, double& az,
	const POSE_EST_PARAMS& params)
{
	// Point clouds
	PointCloudT::Ptr object(new PointCloudT);
	PointCloudT::Ptr object_aligned(new PointCloudT);
	PointCloudT::Ptr scene(new PointCloudT);
	FeatureCloudT::Ptr object_features(new FeatureCloudT);
	FeatureCloudT::Ptr scene_features(new FeatureCloudT);

	PointCloudT::Ptr model_d(new PointCloudT);
	PointCloudT::Ptr scene_d(new PointCloudT);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_d_(new pcl::PointCloud<pcl::PointXYZRGB>);

	PointCloudT::Ptr object_tmp(new PointCloudT);
	PointCloudT::Ptr scene_tmp(new PointCloudT);


	//printf("filtering\n");
	//statistic_outlier_remove_filter(scene_, 30, 0.01, scene_d_);


	pcl::copyPointCloud(model_, *model_d);
	//pcl::copyPointCloud(*scene_d_, *scene_d);
	pcl::copyPointCloud(*scene_, *scene_d);



	// Downsample
	pcl::console::print_highlight("Downsampling...\n");
	pcl::VoxelGrid<PointNT> grid;
	const float leaf = params.leaf;

	grid.setLeafSize(leaf, leaf, leaf);
	grid.setInputCloud(model_d);
	printf("object p num:%d\n", model_d->points.size());
	grid.filter(*model_d);
	printf("object p num:%d\n", model_d->points.size());

	grid.setLeafSize(leaf, leaf, leaf);
	grid.setInputCloud(scene_d);
	printf("scene p num:%d\n", scene_d->points.size());
	grid.filter(*scene_d);
	printf("scene p num:%d\n", scene_d->points.size());



	// Estimate normals for scene
	pcl::console::print_highlight("Estimating normals...\n");
	pcl::NormalEstimationOMP<PointNT, PointNT> nest;
	nest.setRadiusSearch(params.norm_radius);
	nest.setInputCloud(model_d);
	nest.compute(*model_d);
	nest.setInputCloud(scene_d);
	nest.compute(*scene_d);


	printf("pre procs\n");



	// Estimate features
	pcl::console::print_highlight("Estimating features...\n");
	FeatureEstimationT fest;
	fest.setRadiusSearch(params.feature_radius);
	fest.setInputCloud(model_d);
	fest.setInputNormals(model_d);
	fest.compute(*object_features);
	fest.setInputCloud(scene_d);
	fest.setInputNormals(scene_d);
	fest.compute(*scene_features);

	printf("obj f num %d\n", object_features->points.size());
	printf("scn f num %d\n", scene_features->points.size());

	// Perform alignment
	pcl::console::print_highlight("Starting alignment...\n");
	pcl::SampleConsensusPrerejective<PointNT, PointNT, FeatureT> align;
	align.setInputSource(model_d);
	align.setSourceFeatures(object_features);
	align.setInputTarget(scene_d);
	align.setTargetFeatures(scene_features);


	align.setMaximumIterations(params.MaximumIterations);// Number of RANSAC iterations

	align.setNumberOfSamples(params.NumberOfSamples); // Number of points to sample for generating/prerejecting a pose

	align.setSimilarityThreshold(params.SimilarityThreshold);	// Polygonal edge length similarity threshold


	align.setMaxCorrespondenceDistance(params.MaxCorrespondenceDistance);

	align.setCorrespondenceRandomness(params.CorrespondenceRandomness); // Number of nearest features to use
	

	align.setInlierFraction(params.InlierFraction);	// Required inlier fraction for accepting a pose hypothesis
	

	{
		pcl::ScopeTime t("Alignment");
		align.align(*object_aligned);
	}

	if (align.hasConverged())
	{
		// Print results
		printf("\n");
		Eigen::Matrix4f transformation = align.getFinalTransformation();
		print_mat(transformation);
		pcl::console::print_info("\n");
		pcl::console::print_info("Inliers: %i/%i\n", align.getInliers().size(), model_d->size());


		Eigen::Vector4f xyz_centroid;
		pcl::compute3DCentroid(*object_aligned, xyz_centroid);//重心を計算

		computeAnglesFromMatrix(transformation, ax, ay, az);

		cx = xyz_centroid.x();
		cy = xyz_centroid.y();
		cz = xyz_centroid.z();

		pcl::copyPointCloud(*object_aligned, *model_aligned);

		return true;
	}
	else
	{
		pcl::console::print_error("Alignment failed!\n");
		return false;
	}

	return false;
}


bool pose_est(pcl::PointCloud<pcl::PointXYZRGB>& model_,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr model_aligned,
	double& cx, double& cy, double& cz, double &ax, double& ay, double& az)
{
	// Point clouds
	PointCloudT::Ptr object(new PointCloudT);
	PointCloudT::Ptr object_aligned(new PointCloudT);
	PointCloudT::Ptr scene(new PointCloudT);
	FeatureCloudT::Ptr object_features(new FeatureCloudT);
	FeatureCloudT::Ptr scene_features(new FeatureCloudT);

	PointCloudT::Ptr model_d(new PointCloudT);
	PointCloudT::Ptr scene_d(new PointCloudT);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_d_(new pcl::PointCloud<pcl::PointXYZRGB>);

	PointCloudT::Ptr object_tmp(new PointCloudT);
	PointCloudT::Ptr scene_tmp(new PointCloudT);


	//printf("filtering\n");
	//statistic_outlier_remove_filter(scene_, 30, 0.01, scene_d_);


	pcl::copyPointCloud(model_, *model_d);
	//pcl::copyPointCloud(*scene_d_, *scene_d);
	pcl::copyPointCloud(*scene_, *scene_d);



	// Downsample
	pcl::console::print_highlight("Downsampling...\n");
	pcl::VoxelGrid<PointNT> grid;
	//const float leaf = 0.0075f;
	//const float leaf = 0.005f;
	const float leaf = 0.0025f;	//●
	//const float leaf = 0.00125f;
	grid.setLeafSize(leaf, leaf, leaf);
	grid.setInputCloud(model_d);
	printf("object p num:%d\n", model_d->points.size());
	grid.filter(*model_d);
	printf("object p num:%d\n", model_d->points.size());

	grid.setLeafSize(leaf, leaf, leaf);
	grid.setInputCloud(scene_d);
	printf("scene p num:%d\n", scene_d->points.size());
	grid.filter(*scene_d);
	printf("scene p num:%d\n", scene_d->points.size());



	// Estimate normals for scene
	pcl::console::print_highlight("Estimating normals...\n");
	pcl::NormalEstimationOMP<PointNT, PointNT> nest;
	nest.setRadiusSearch(0.01);
	nest.setInputCloud(model_d);
	nest.compute(*model_d);
	nest.setInputCloud(scene_d);
	nest.compute(*scene_d);


	printf("pre procs\n");



	// Estimate features
	pcl::console::print_highlight("Estimating features...\n");
	FeatureEstimationT fest;
	fest.setRadiusSearch(0.025);
	fest.setInputCloud(model_d);
	fest.setInputNormals(model_d);
	fest.compute(*object_features);
	fest.setInputCloud(scene_d);
	fest.setInputNormals(scene_d);
	fest.compute(*scene_features);

	printf("obj f num %d\n", object_features->points.size());
	printf("scn f num %d\n", scene_features->points.size());

	// Perform alignment
	pcl::console::print_highlight("Starting alignment...\n");
	pcl::SampleConsensusPrerejective<PointNT, PointNT, FeatureT> align;
	align.setInputSource(model_d);
	align.setSourceFeatures(object_features);
	align.setInputTarget(scene_d);
	align.setTargetFeatures(scene_features);

	//align.setMaximumIterations(50000); // Number of RANSAC iterations
	//align.setMaximumIterations(350000);
	align.setMaximumIterations(50000);

	align.setNumberOfSamples(3); // Number of points to sample for generating/prerejecting a pose
	//align.setNumberOfSamples(5);

	//align.setSimilarityThreshold(0.9f); // Polygonal edge length similarity threshold
	//align.setSimilarityThreshold(0.6f);
	//align.setSimilarityThreshold(0.7f);
	//align.setSimilarityThreshold(0.8f);
	//align.setSimilarityThreshold(0.75f);
	//align.setSimilarityThreshold(0.7f);
	//align.setSimilarityThreshold(0.65f);
	align.setSimilarityThreshold(0.6f);


										//align.setMaxCorrespondenceDistance(2.5f * leaf); // Inlier threshold
										//align.setMaxCorrespondenceDistance(2.5f * leaf);
	
										//align.setMaxCorrespondenceDistance(2.0f * leaf);
	//align.setMaxCorrespondenceDistance(0.005);
	align.setMaxCorrespondenceDistance(3.0f * leaf);
	//align.setMaxCorrespondenceDistance(0.01);

	//align.setCorrespondenceRandomness(5); // Number of nearest features to use
	//align.setCorrespondenceRandomness(30);

	//align.setInlierFraction(0.1f);
	align.setInlierFraction(0.15f);
	//align.setInlierFraction(0.2f);
	//align.setInlierFraction(0.25f); // Required inlier fraction for accepting a pose hypothesis
	//align.setInlierFraction(0.4f);
	//align.setInlierFraction(0.5f);


	{
		pcl::ScopeTime t("Alignment");
		align.align(*object_aligned);
	}

	if (align.hasConverged())
	{
		// Print results
		printf("\n");
		Eigen::Matrix4f transformation = align.getFinalTransformation();
		pcl::console::print_info("    | %6.3f %6.3f %6.3f | \n", transformation(0, 0), transformation(0, 1), transformation(0, 2));
		pcl::console::print_info("R = | %6.3f %6.3f %6.3f | \n", transformation(1, 0), transformation(1, 1), transformation(1, 2));
		pcl::console::print_info("    | %6.3f %6.3f %6.3f | \n", transformation(2, 0), transformation(2, 1), transformation(2, 2));
		pcl::console::print_info("\n");
		pcl::console::print_info("t = < %0.3f, %0.3f, %0.3f >\n", transformation(0, 3), transformation(1, 3), transformation(2, 3));
		pcl::console::print_info("\n");
		pcl::console::print_info("Inliers: %i/%i\n", align.getInliers().size(), model_d->size());

		// Show alignment
		//pcl::visualization::PCLVisualizer visu("Alignment");
		//visu.addPointCloud(scene_d, ColorHandlerT(scene_d, 0.0, 255.0, 0.0), "scene_d");
		//visu.addPointCloud(scene_d, ColorHandlerRGB(scene_d), "scene_d");

		//visu.addPointCloud(model_d, ColorHandlerT(model_d, 255.0, 0.0, 255.0), "model_d");
		//visu.addPointCloud(model_d, ColorHandlerRGB(model_d), "model_d");

		//visu.addPointCloud(object_aligned, ColorHandlerT(object_aligned, 255.0, 0.0, 255.0), "object_aligned");
		//visu.addPointCloud(object_aligned, ColorHandlerRGB(object_aligned), "object_aligned");


		//visu.addPointCloud(object, ColorHandlerT(object, 255.0, 255.0, 255.0), "object");
		//visu.addPointCloud(scene, ColorHandlerT(scene, 255.0, 255.0, 255.0), "scene");


		//visu.setBackgroundColor(255, 255, 255);

		//visu.spin();


		Eigen::Vector4f xyz_centroid;
		pcl::compute3DCentroid(*object_aligned, xyz_centroid);//重心を計算

		computeAnglesFromMatrix(transformation, ax, ay, az);

		cx = xyz_centroid.x();
		cy = xyz_centroid.y();
		cz = xyz_centroid.z();

		pcl::copyPointCloud(*object_aligned, *model_aligned);

		return true;
	}
	//else
	{
		pcl::console::print_error("Alignment failed!\n");
		return false;
	}

	return false;
}




bool pose_est2(pcl::PointCloud<pcl::PointXYZRGB>& model__,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr model_aligned,
	double& cx, double& cy, double& cz, double &ax, double& ay, double& az)
{
	// Point clouds
	PointCloudT::Ptr object(new PointCloudT);
	PointCloudT::Ptr object_aligned(new PointCloudT);
	PointCloudT::Ptr scene(new PointCloudT);
	FeatureCloudT::Ptr object_features(new FeatureCloudT);
	FeatureCloudT::Ptr scene_features(new FeatureCloudT);

	PointCloudT::Ptr model_d(new PointCloudT);
	PointCloudT::Ptr scene_d(new PointCloudT);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_d_(new pcl::PointCloud<pcl::PointXYZRGB>);

	PointCloudT::Ptr object_tmp(new PointCloudT);
	PointCloudT::Ptr scene_tmp(new PointCloudT);


	//printf("filtering\n");
	//statistic_outlier_remove_filter(scene_, 30, 0.01, scene_d_);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr model_(new pcl::PointCloud<pcl::PointXYZRGB>);
	*model_ = model__;


	pcl::copyPointCloud(*model_, *model_d);
	//pcl::copyPointCloud(*scene_d_, *scene_d);
	pcl::copyPointCloud(*scene_, *scene_d);



	// Downsample
	pcl::console::print_highlight("Downsampling...\n");
	pcl::VoxelGrid<PointNT> grid;
	//const float leaf = 0.0075f;
	//const float leaf = 0.005f;
	const float leaf = 0.0025f;	//●
								//const float leaf = 0.00125f;
	grid.setLeafSize(leaf, leaf, leaf);
	grid.setInputCloud(model_d);
	printf("object p num:%d\n", model_d->points.size());
	grid.filter(*model_d);
	printf("object p num:%d\n", model_d->points.size());

	grid.setLeafSize(leaf, leaf, leaf);
	grid.setInputCloud(scene_d);
	printf("scene p num:%d\n", scene_d->points.size());
	grid.filter(*scene_d);
	printf("scene p num:%d\n", scene_d->points.size());



	// Estimate normals for scene
	pcl::console::print_highlight("Estimating normals...\n");
	pcl::NormalEstimationOMP<PointNT, PointNT> nest;
	nest.setRadiusSearch(0.01);
	nest.setInputCloud(model_d);
	nest.compute(*model_d);
	nest.setInputCloud(scene_d);
	nest.compute(*scene_d);


	printf("pre procs\n");






	pcl::PointCloud<pcl::PointXYZRGB>::Ptr model_dn(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_dn(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::copyPointCloud(*model_d, *model_dn);
	pcl::copyPointCloud(*scene_d, *scene_dn);





	//キーポイント探索
	printf("keypoints\n");
	pcl::HarrisKeypoint6D<pcl::PointXYZRGB, pcl::PointXYZI>* harris6D = new pcl::HarrisKeypoint6D<pcl::PointXYZRGB, pcl::PointXYZI>();	//
	harris6D->setNonMaxSupression(true);
	harris6D->setRadius(0.001);// 3);
	harris6D->setRadiusSearch(0.001);// 3);
	boost::shared_ptr<pcl::Keypoint<pcl::PointXYZRGB, pcl::PointXYZI> > keypoint_detector;
	keypoint_detector.reset(harris6D);

	pcl::PointCloud<pcl::PointXYZI>::Ptr  model_kp(new pcl::PointCloud<pcl::PointXYZI>);
	detectKeypoints(keypoint_detector, model_dn, model_kp);
	printf("model keypoints num : %d\n", model_kp->points.size());


	pcl::PointCloud<pcl::PointXYZI>::Ptr  target_kp(new pcl::PointCloud<pcl::PointXYZI>);
	detectKeypoints(keypoint_detector, scene_dn, target_kp);
	printf("target keypoints num : %d\n", target_kp->points.size());



	//記述子計算
	pcl::SHOTColorEstimationOMP<pcl::PointXYZRGB, pcl::Normal, pcl::SHOT1344>* shot = new pcl::SHOTColorEstimationOMP<pcl::PointXYZRGB, pcl::Normal, pcl::SHOT1344>;
	shot->setRadiusSearch(0.04);
	pcl::Feature<pcl::PointXYZRGB, pcl::SHOT1344>::Ptr feature_extractor(shot);

	pcl::PointCloud<pcl::SHOT1344>::Ptr model_ft(new pcl::PointCloud<pcl::SHOT1344>);
	extractDescriptors(feature_extractor, model_dn, model_kp, model_ft);

	pcl::PointCloud<pcl::SHOT1344>::Ptr target_ft(new pcl::PointCloud<pcl::SHOT1344>);
	extractDescriptors(feature_extractor, scene_dn, target_kp, target_ft);


	//対応付け計算
	std::vector<int> correspondences_m2t, correspondences_t2m;
	findCorrespondences(model_ft, target_ft, correspondences_m2t);
	findCorrespondences(target_ft, model_ft, correspondences_t2m);
	printf("corrd m2t:%d\n", correspondences_m2t.size());
	printf("corrd t2m:%d\n", correspondences_t2m.size());

	//対応付けのフィルタリング
	pcl::CorrespondencesPtr filtered_correspondences(new pcl::Correspondences);
	filterCorrespondences(model_kp, target_kp,
		correspondences_m2t, correspondences_t2m, filtered_correspondences);
	printf("corrd filtered:%d\n", filtered_correspondences->size());

	////最終的な、初期変換値の計算
	Eigen::Matrix4f initial_transformation_matrix_;
	PointCloudT::Ptr model_cl_initial_transformed_(new PointCloudT);
	determineInitialTransformation(model_d, filtered_correspondences, model_kp, target_kp,
		initial_transformation_matrix_);

	printf("init mat\n");
	print_mat(initial_transformation_matrix_);



	// Estimate features
	pcl::console::print_highlight("Estimating features...\n");
	FeatureEstimationT fest;
	fest.setRadiusSearch(0.025);
	fest.setInputCloud(model_d);
	fest.setInputNormals(model_d);
	fest.compute(*object_features);
	fest.setInputCloud(scene_d);
	fest.setInputNormals(scene_d);
	fest.compute(*scene_features);

	printf("obj f num %d\n", object_features->points.size());
	printf("scn f num %d\n", scene_features->points.size());

	// Perform alignment
	pcl::console::print_highlight("Starting alignment...\n");
	pcl::SampleConsensusPrerejective<PointNT, PointNT, FeatureT> align;
	align.setInputSource(model_d);
	align.setSourceFeatures(object_features);
	align.setInputTarget(scene_d);
	align.setTargetFeatures(scene_features);

	//align.setMaximumIterations(50000); // Number of RANSAC iterations
	align.setMaximumIterations(350000);

	align.setNumberOfSamples(3); // Number of points to sample for generating/prerejecting a pose
								 //align.setNumberOfSamples(5);

	align.setSimilarityThreshold(0.9f); // Polygonal edge length similarity threshold
										//align.setSimilarityThreshold(0.95f);


										//align.setMaxCorrespondenceDistance(2.5f * leaf); // Inlier threshold
										//align.setMaxCorrespondenceDistance(2.5f * leaf);

										//align.setMaxCorrespondenceDistance(2.0f * leaf);
	align.setMaxCorrespondenceDistance(0.005);
	//align.setMaxCorrespondenceDistance(3.0f * leaf);
	//align.setMaxCorrespondenceDistance(0.01);

	align.setCorrespondenceRandomness(5); // Number of nearest features to use
										  //align.setCorrespondenceRandomness(30);

										  //align.setInlierFraction(0.1f);
	align.setInlierFraction(0.2f);
	//align.setInlierFraction(0.25f); // Required inlier fraction for accepting a pose hypothesis
	//align.setInlierFraction(0.4f);
	//align.setInlierFraction(0.5f);


	{
		pcl::ScopeTime t("Alignment");
		//align.align(*object_aligned);

		align.align(*object_aligned, initial_transformation_matrix_);
	}

	if (align.hasConverged())
	{
		// Print results
		printf("\n");
		Eigen::Matrix4f transformation = align.getFinalTransformation();
		print_mat(transformation);
		pcl::console::print_info("\n");
		pcl::console::print_info("Inliers: %i/%i\n", align.getInliers().size(), model_d->size());

		// Show alignment
		//pcl::visualization::PCLVisualizer visu("Alignment");
		//visu.addPointCloud(scene_d, ColorHandlerT(scene_d, 0.0, 255.0, 0.0), "scene_d");
		//visu.addPointCloud(scene_d, ColorHandlerRGB(scene_d), "scene_d");

		//visu.addPointCloud(model_d, ColorHandlerT(model_d, 255.0, 0.0, 255.0), "model_d");
		//visu.addPointCloud(model_d, ColorHandlerRGB(model_d), "model_d");

		//visu.addPointCloud(object_aligned, ColorHandlerT(object_aligned, 255.0, 0.0, 255.0), "object_aligned");
		//visu.addPointCloud(object_aligned, ColorHandlerRGB(object_aligned), "object_aligned");


		//visu.addPointCloud(object, ColorHandlerT(object, 255.0, 255.0, 255.0), "object");
		//visu.addPointCloud(scene, ColorHandlerT(scene, 255.0, 255.0, 255.0), "scene");


		//visu.setBackgroundColor(255, 255, 255);

		//visu.spin();


		Eigen::Vector4f xyz_centroid;
		pcl::compute3DCentroid(*object_aligned, xyz_centroid);//重心を計算

		computeAnglesFromMatrix(transformation, ax, ay, az);

		cx = xyz_centroid.x();
		cy = xyz_centroid.y();
		cz = xyz_centroid.z();

		pcl::copyPointCloud(*object_aligned, *model_aligned);

		return true;
	}
	else
	{
		pcl::console::print_error("Alignment failed!\n");
		return false;
	}

	return false;
}
