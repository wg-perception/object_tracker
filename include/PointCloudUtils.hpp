/*
 * PointCloudUtils.hpp
 *
 *  Created on: Nov 6, 2012
 *      Author: tcavallari
 */
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PointIndices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>

#ifndef POINTCLOUDUTILS_HPP_
#define POINTCLOUDUTILS_HPP_

template<typename PointType>
class PointCloudUtils
{
public:
	typedef pcl::PointCloud<PointType> PointCloud;
	typedef typename PointCloud::ConstPtr PointCloudConstPtr;

	static void filterPassThrough(const PointCloudConstPtr& cloud,
			PointCloud& result)
	{
		pcl::PassThrough<PointType> pass;
		pass.setFilterFieldName("z");
		pass.setFilterLimits(0.0, 10.0);
		//pass.setFilterLimits (0.0, 1.5);
		//pass.setFilterLimits (0.0, 0.6);
		pass.setKeepOrganized(false);
		pass.setInputCloud(cloud);
		pass.filter(result);
	}

	static void removeZeroPoints(const PointCloudConstPtr& cloud,
			PointCloud& result)
	{
		result.clear();

		for (size_t i = 0; i < cloud->points.size(); i++)
		{
			PointType point = cloud->points[i];
			if (/*!(fabs(point.x) < 0.01 && fabs(point.y) < 0.01
					&& fabs(point.z) < 0.01) &&*/ !pcl_isnan(point.x)&&
			!pcl_isnan(point.y) &&
			!pcl_isnan(point.z))
			result.points.push_back(point);
		}

		result.width = static_cast<uint32_t>(result.points.size());
		result.height = 1;
		result.is_dense = true;
	}

	static void gridSample(const PointCloudConstPtr& cloud, PointCloud& result,
			double leaf_size = 0.005)
	{
		pcl::VoxelGrid<PointType> grid;
		grid.setLeafSize(float(leaf_size), float(leaf_size), float(leaf_size));
		grid.setInputCloud(cloud);
		grid.filter(result);
	}

	static void gridSampleApprox(const PointCloudConstPtr& cloud,
			PointCloud& result, double leaf_size = 0.005)
	{
		pcl::ApproximateVoxelGrid<PointType> grid;
		grid.setLeafSize(static_cast<float>(leaf_size),
				static_cast<float>(leaf_size), static_cast<float>(leaf_size));
		grid.setInputCloud(cloud);
		grid.filter(result);
	}

	static void planeSegmentation(const PointCloudConstPtr& cloud,
			pcl::ModelCoefficients& coefficients, pcl::PointIndices& inliers)
	{
		pcl::SACSegmentation<PointType> seg;
		seg.setOptimizeCoefficients(true);
		seg.setModelType(pcl::SACMODEL_PLANE);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setMaxIterations(1000);
		seg.setDistanceThreshold(0.015);
		seg.setInputCloud(cloud);
		seg.segment(inliers, coefficients);
	}

	static void planeProjection(const PointCloudConstPtr& cloud,
			const pcl::ModelCoefficients::ConstPtr& coefficients,
			const pcl::PointIndices::ConstPtr& inliers,
			PointCloud &result)
	{
		pcl::ProjectInliers<PointType> proj;
		proj.setModelType(pcl::SACMODEL_PLANE);
		proj.setInputCloud(cloud);
		proj.setIndices(inliers);
		proj.setModelCoefficients(coefficients);
		proj.filter(result);
	}

	static void convexHull(const PointCloudConstPtr& cloud,
			PointCloud& cloud_hull, std::vector<pcl::Vertices>& hull_vertices)
	{
		pcl::ConvexHull<PointType> chull;
		chull.setInputCloud(cloud);
		chull.setDimension(2);
		chull.reconstruct(cloud_hull, hull_vertices);
	}

	static void extractNonPlanePoints(const PointCloudConstPtr& cloud,
			const PointCloudConstPtr& cloud_hull, PointCloud& result)
	{
		pcl::ExtractPolygonalPrismData<PointType> polygon_extract;
		pcl::PointIndices::Ptr inliers_polygon(new pcl::PointIndices());
		polygon_extract.setHeightLimits(0.01, 10.0);
		polygon_extract.setInputPlanarHull(cloud_hull);
		polygon_extract.setInputCloud(cloud);
		polygon_extract.segment(*inliers_polygon);

		pcl::ExtractIndices<PointType> extract_positive;
		extract_positive.setNegative(false);
		extract_positive.setInputCloud(cloud);
		extract_positive.setIndices(inliers_polygon);
		extract_positive.filter(result);
	}

	static void euclideanSegment(const PointCloudConstPtr& cloud,
			std::vector<pcl::PointIndices>& cluster_indices)
	{
		pcl::EuclideanClusterExtraction<PointType> ec;
		ec.setClusterTolerance(0.03); // 2cm
		ec.setMinClusterSize(50);
		ec.setMaxClusterSize(25000);
		//ec.setMaxClusterSize (400);
		ec.setInputCloud(cloud);
		ec.extract(cluster_indices);
	}

	static void extractSegmentCluster(const PointCloudConstPtr& cloud,
			const std::vector<pcl::PointIndices>& cluster_indices,
			const int segment_index, PointCloud& result)
	{
		pcl::PointIndices& segmented_indices = cluster_indices[segment_index];
		result.clear();

		for (size_t i = 0; i < segmented_indices.indices.size(); i++)
		{
			PointType point = cloud->points[segmented_indices.indices[i]];
			result.points.push_back(point);
		}
		result.width = static_cast<uint32_t>(result.points.size());
		result.height = 1;
		result.is_dense = true;
	}

	static void extractCluster(const PointCloudConstPtr& cloud,
			const pcl::PointIndices& cluster_indices, PointCloud& result)
	{
		pcl::ExtractIndices<PointType> extract_positive;
		extract_positive.setNegative(false);
		extract_positive.setInputCloud(cloud);
		extract_positive.setIndices(boost::make_shared<const pcl::PointIndices>(cluster_indices));
		extract_positive.filter(result);
	}

	static void organizedMultiplaneSegmentation(const PointCloudConstPtr& cloud, PointCloud& cloud_no_plane)
	{
		pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>());
		pcl::IntegralImageNormalEstimation<PointType, pcl::Normal> ne;
		ne.setInputCloud(cloud);
		ne.compute(*normals);

		pcl::OrganizedMultiPlaneSegmentation<PointType, pcl::Normal, pcl::Label> plane_segmentation;
		plane_segmentation.setInputCloud(cloud);
		plane_segmentation.setInputNormals(normals);
		//plane_segmentation.setDistanceThreshold(0.02);
		//plane_segmentation.setAngularThreshold(pcl::deg2rad(5.0));
		//plane_segmentation.setMaximumCurvature(0.01);
		plane_segmentation.setProjectPoints(true);

		std::vector<pcl::PlanarRegion<PointType>, Eigen::aligned_allocator<pcl::PlanarRegion<PointType> > > regions;
		std::vector<pcl::ModelCoefficients> model_coefficients;
		std::vector<pcl::PointIndices> inlier_indices;
		pcl::PointCloud<pcl::Label>::Ptr labels (new pcl::PointCloud<pcl::Label>());
		std::vector<pcl::PointIndices> label_indices;
		std::vector<pcl::PointIndices> boundary_indices;

		plane_segmentation.segmentAndRefine(regions, model_coefficients, inlier_indices, labels, label_indices, boundary_indices);

		boost::shared_ptr<std::vector<int> > inliers (new std::vector<int>());
		for (int i = 0; i < inlier_indices.size(); ++i) {
			inliers->insert(inliers->end(), inlier_indices[i].indices.begin(), inlier_indices[i].indices.end());
		}

		pcl::ExtractIndices<PointType> extract_indices;
		extract_indices.setInputCloud(cloud);
		extract_indices.setIndices(inliers);
		extract_indices.setNegative(true);
		extract_indices.filter(cloud_no_plane);
	}

};

#endif /* POINTCLOUDUTILS_HPP_ */
