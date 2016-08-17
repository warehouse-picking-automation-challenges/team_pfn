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

#include "plane_funcs.h"

bool detect_largest_plane(
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cld,
	double det_plane_dist,
	pcl::PointIndices::Ptr out_plane_index,
	pcl::ModelCoefficients::Ptr out_coeffi)
{

	pcl::SACSegmentation<pcl::PointXYZRGB> seg;

	seg.setOptimizeCoefficients(true);

	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(det_plane_dist);	//0.01など

	seg.setInputCloud(in_cld);
	seg.segment(*out_plane_index, *out_coeffi);


	if (out_plane_index->indices.size() > 0)
		return true;
	else
		return false;
}

void extract_or_delete_by_index(
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cld,
	pcl::PointIndices::Ptr in_index,
	bool proc_is_delete,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cld)
{
	pcl::ExtractIndices<pcl::PointXYZRGB> ext;
	ext.setInputCloud(in_cld);
	ext.setIndices(in_index);
	ext.setNegative(proc_is_delete);	//true で削除
	ext.filter(*out_cld);
}


void extract_biggest_object_on_plane(
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cld,
	double det_plane_dist,
	int min_obj_pt_num,
	int max_obj_pt_num,
	double obj_clst_dist,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cld)
{

	pcl::ModelCoefficients::Ptr pln_coef(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr pln_index(new pcl::PointIndices);


	detect_largest_plane(in_cld, det_plane_dist, pln_index, pln_coef);

	extract_or_delete_by_index(in_cld, pln_index, true, out_cld);

	std::vector<int> indices;
	pcl::removeNaNFromPointCloud(*out_cld, *out_cld, indices);
	

	// 残りの物体のうち、大きいもののみ選択
	typename pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
	tree->setInputCloud(out_cld);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> clustering;
	clustering.setClusterTolerance(obj_clst_dist); // 2cm
	clustering.setMinClusterSize(min_obj_pt_num);
	clustering.setMaxClusterSize(max_obj_pt_num);
	clustering.setSearchMethod(tree);
	clustering.setInputCloud(out_cld);
	clustering.extract(cluster_indices);

	if (cluster_indices.size() > 0)
	{
		pcl::PointIndices::Ptr id(new pcl::PointIndices);
		id->indices = cluster_indices[0].indices;	//無駄なコピー。。  index=0 largest
		extract_or_delete_by_index(out_cld, id, false, out_cld);
	}
	else
	{
		out_cld->clear();
	}

}

void extract_objects(
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cld,
	int min_obj_pt_num,
	int max_obj_pt_num,
	double obj_clst_dist,
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& out_cld_list)
{


	// 物体のうち、大きいもののみ選択
	typename pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
	tree->setInputCloud(in_cld);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> clustering;
	clustering.setClusterTolerance(obj_clst_dist); // 2cm
	clustering.setMinClusterSize(min_obj_pt_num);
	clustering.setMaxClusterSize(max_obj_pt_num);
	clustering.setSearchMethod(tree);
	clustering.setInputCloud(in_cld);
	clustering.extract(cluster_indices);

	printf("%d\n", cluster_indices.size());
	out_cld_list.clear();

	for (int i = 0; i < cluster_indices.size(); i++)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmpcld(new pcl::PointCloud<pcl::PointXYZRGB>);

		tmpcld->points.reserve(cluster_indices[i].indices.size());

		for (int j = 0; j < cluster_indices[i].indices.size(); j++)
		{
			tmpcld->push_back(in_cld->points[cluster_indices[i].indices[j]]);
		}
		tmpcld->width = tmpcld->points.size();
		tmpcld->height = 1;
		tmpcld->is_dense = false;

		out_cld_list.push_back(tmpcld);

		printf("%d\n", tmpcld->width);

	}

}


void extract_bigger_objects(
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cld,
	int min_obj_pt_num,
	int max_obj_pt_num,
	double obj_clst_dist,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cld)
{
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> out_cld_list;
	extract_objects(in_cld, min_obj_pt_num, max_obj_pt_num, obj_clst_dist, out_cld_list);

	printf("%d\n", out_cld_list.front()->size());

	*out_cld = *out_cld_list.front();	//たぶん一番大きい（ソートされているはず）
}