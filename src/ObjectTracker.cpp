/*
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 * PointCloudClusterer.hpp
 *
 *  Created on: Nov 6, 2012
 *      Author: Tommaso Cavallari
 */

#include "ObjectTracker.hpp"
#include "PointCloudUtils.hpp"
#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/common/transforms.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/octree.h>
#include <pcl/tracking/particle_filter.h>
#include <pcl/tracking/particle_filter_omp.h>
#include <pcl/tracking/kld_adaptive_particle_filter.h>
#include <pcl/tracking/kld_adaptive_particle_filter_omp.h>
#include <pcl/tracking/coherence.h>
#include <pcl/tracking/distance_coherence.h>
#include <pcl/tracking/hsv_color_coherence.h>
#include <pcl/tracking/normal_coherence.h>

#include <pcl/tracking/approx_nearest_pair_point_cloud_coherence.h>
#include <pcl/tracking/nearest_pair_point_cloud_coherence.h>

int main(int argc, char** argv)
{
	// register the node
	ros::init(argc, argv, "object_tracker", ros::init_options::AnonymousName);

	ObjectTrackerNode tracker;
	ros::CallbackQueue object_recognition_queue;

	bool ok = tracker.init(&object_recognition_queue);

	if (!ok)
		exit(-1);

	// 1 thread
	ros::AsyncSpinner object_recognition_spinner(1, &object_recognition_queue);
	object_recognition_spinner.start();
	ros::spin();
	return 0;
}

bool ObjectTrackerNode::init(
		ros::CallbackQueueInterface *object_recognition_queue)
{
	nh_object_recognition_.setCallbackQueue(object_recognition_queue);

//	boost::function<void(const object_recognition_msgs::RecognizedObjectArrayConstPtr&, const PointCloud::ConstPtr&)> recognition_callback(
//			boost::bind(&ObjectTrackerNode::recognizedObjectCallback, this,
//					_1, _2));
//	object_recognition_subscriber_ = nh_object_recognition_.subscribe<
//			object_recognition_msgs::RecognizedObjectArrayConstPtr>(
//			"recognized_object_array", 1, recognition_callback);

	synchronizer_.registerCallback(&ObjectTrackerNode::recognizedObjectCallback, this);
//	synchronizer_.registerCallback(boost::bind(&ObjectTrackerNode::recognizedObjectCallback, this, _1, _2));

	boost::function<void(PointCloud::ConstPtr)> cloud_callback(
			boost::bind(&ObjectTrackerNode::cloudCallback, this, _1));
	cloud_subscriber_ = nh_.subscribe<PointCloud::ConstPtr>("cloud_in", 5,
			cloud_callback);

	object_pose_publisher_ = nh_.advertise<geometry_msgs::PoseArray>(
			"tracked_poses", 1);
	cloud_no_plane_publisher_ = nh_.advertise<PointCloud>(
			"cloud_no_plane", 1);
	particles_publisher_ = nh_.advertise<StateCloud>(
			"cloud_particles", 1);
	recognized_object_array_publisher_ = nh_.advertise<object_recognition_msgs::RecognizedObjectArray>("tracked_objects", 1);
	object_names_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>("object_names", 1);

	//segment_tracker_.run();

//	ros::NodeHandle priv_nh("~");
//	priv_nh.getParam("frame_id", frame_id_);

	return true;
}

void ObjectTrackerNode::recognizedObjectCallback(
		const object_recognition_msgs::RecognizedObjectArrayConstPtr& msg_recognition, const PointCloud::ConstPtr& cloud)
{
	PointCloud::Ptr downsampled_cloud(new PointCloud());
//	PointCloud::ConstPtr old_cloud(new PointCloud());

	ROS_INFO(
			"Received a callback with %zu recognitions.", msg_recognition->objects.size());

//	{
//		boost::mutex::scoped_lock prev_cloud_lock(previous_cloud_mutex_);
//		old_cloud = previous_cloud_;
//	}
//
//	if (!old_cloud)
//	{
//		ROS_INFO("Received a detection callback but cloud never received.");
//		return;
//	}

//	ROS_INFO("Delta: %f Old timestamp: %f New Timestamp: %f", cloud->header.stamp.toSec() - old_cloud->header.stamp.toSec(), old_cloud->header.stamp.toSec(), cloud->header.stamp.toSec());
//	if(fabs(cloud->header.stamp.toSec() - old_cloud->header.stamp.toSec()) > 2.5)
//	{
//		ROS_WARN("Running slow, delta T > 2.5s, returning.");
//		return;
//	}

	{
		ros::Time now (ros::Time::now());
		double delta_t = (now - cloud->header.stamp).toSec();

		ROS_INFO("Delta: %f Cloud timestamp: %f Now Timestamp: %f", delta_t, cloud->header.stamp.toSec(), now.toSec());
		if(delta_t > 2.5)
		{
			ROS_WARN("Running slow, delta T > 2.5s, returning.");
			return;
		}
	}

	// Copy lists into local scope
	std::list<TrackedObjectPtr> new_objects_list_copy;
	std::list<TrackedObjectPtr> regular_objects_list_copy;
	std::list<TrackedObjectPtr> stale_objects_list_copy;
	{
		boost::mutex::scoped_lock lists_lock(lists_mutex_);
		new_objects_list_copy = new_objects_list_;
		regular_objects_list_copy = regular_objects_list_;
		stale_objects_list_copy = stale_objects_list_;
	}

	// If there are no recognitions...
	if (msg_recognition->objects.size() == 0)
	{
		// Everything regular becomes stale
		// Increase the age of the new entries
		// if needed delete some new/stale entries
		// publish poses
		std::list<TrackedObjectPtr>::iterator iter =
				regular_objects_list_copy.begin();
		while (iter != regular_objects_list_copy.end())
		{
			TrackedObjectPtr cur_obj = *iter;
			iter = regular_objects_list_copy.erase(iter);

			cur_obj->age()++;cur_obj
			->staleCount() = 0;

			stale_objects_list_copy.push_back(cur_obj);
		}

		// Increase age of new elems
		iter = new_objects_list_copy.begin();
		while (iter != new_objects_list_copy.end())
		{
			(*iter)->age()++;
			(*iter)->newCount()++;

			if ((*iter)->newCount() > new_threshold_)
			{
				iter = new_objects_list_copy.erase(iter);
			}
			else
			{
				++iter;
			}
		}

		// Increase age of stale elems (including fresh stales)
		iter = stale_objects_list_copy.begin();
		while (iter != stale_objects_list_copy.end())
		{
			(*iter)->age()++;
			(*iter)->staleCount()++;

			if ((*iter)->staleCount() > stale_threshold_)
			{
				iter = stale_objects_list_copy.erase(iter);
			}
			else
			{
				++iter;
			}
		}

		//
		// Publish poses (if needed)
		//
		publishPoses(cloud->header.frame_id, cloud->header.stamp, new_objects_list_copy, regular_objects_list_copy,
				stale_objects_list_copy);

		{
			boost::mutex::scoped_lock lists_lock(lists_mutex_);
			new_objects_list_ = new_objects_list_copy;
			regular_objects_list_ = regular_objects_list_copy;
			stale_objects_list_ = stale_objects_list_copy;
		}

		return;
	}

	ROS_INFO("The cloud frame id is %s, the recognition frame_id is %s. The cloud is organized: %s.", cloud->header.frame_id.c_str(), msg_recognition->objects[0].header.frame_id.c_str(), cloud->isOrganized() ? "true" : "false");
	Utils::gridSampleApprox(cloud, *downsampled_cloud);

	//
	// First thing, check into the current tracking sets
	//
	std::vector<bool> recognition_solved(msg_recognition->objects.size(),
			false);
	std::vector<TrackedObjectPtr> object_for_recognition(
			msg_recognition->objects.size());

	for (size_t rec_idx = 0; rec_idx < msg_recognition->objects.size();
			++rec_idx)
	{
		const object_recognition_msgs::RecognizedObject& current_recognition =
				msg_recognition->objects[rec_idx];
		const pcl::PointXYZ current_recognition_centroid(
				current_recognition.pose.pose.pose.position.x,
				current_recognition.pose.pose.pose.position.y,
				current_recognition.pose.pose.pose.position.z);
		const object_recognition_msgs::ObjectId current_recognition_id =
				current_recognition.id;

		// Find nearest centroid for tracking list (with a max_distance threshold)
		// choose nearest one (greedy) that has the same object_id
		float min_new_distance = same_object_threshold_;
		float min_regular_distance = same_object_threshold_;
		float min_stale_distance = same_object_threshold_;

		TrackedObjectPtr min_new_ptr;
		TrackedObjectPtr min_regular_ptr;
		TrackedObjectPtr min_stale_ptr;

		StateType object_centroid;
		ros::Time object_stamp;

		// New list
		BOOST_FOREACH(TrackedObjectPtr object, new_objects_list_copy)
		{
			if (current_recognition_id != object->getObjectId())
			{
				continue;
			}

			if(!object->getPoseAt(cloud->header.stamp, object_centroid, object_stamp))
			{
				ROS_ERROR("Object does not have previous poses, this should not happen.");
				return;
			}

			if(fabs((object_stamp - cloud->header.stamp).toSec()) > 1.0)
			{
				ROS_WARN("Delta time between recognition and tracking in the new list: %f Object stamp: %f Cloud stamp: %f", (object_stamp - cloud->header.stamp).toSec(), object_stamp.toSec(), cloud->header.stamp.toSec());
			}

			float distance = pcl::euclideanDistance(
					current_recognition_centroid, object_centroid);
			if (distance < min_new_distance)
			{
				min_new_distance = distance;
				min_new_ptr = object;
			}
		}

		// Regular list
		BOOST_FOREACH(TrackedObjectPtr object, regular_objects_list_copy)
		{
			if (current_recognition_id != object->getObjectId())
			{
				continue;
			}

			if(!object->getPoseAt(cloud->header.stamp, object_centroid, object_stamp))
			{
				ROS_ERROR("Object does not have previous poses, this should not happen.");
				return;
			}

			if(fabs((object_stamp - cloud->header.stamp).toSec()) > 1.0)
			{
				ROS_WARN("Delta time between recognition and tracking in the regular list: %f", (object_stamp - cloud->header.stamp).toSec());
			}

			float distance = pcl::euclideanDistance(
					current_recognition_centroid, object_centroid);
			if (distance < min_regular_distance)
			{
				min_regular_distance = distance;
				min_regular_ptr = object;
			}
		}

		// Stale list
		BOOST_FOREACH(TrackedObjectPtr object, stale_objects_list_copy)
		{
			if (current_recognition_id != object->getObjectId())
			{
				continue;
			}

			if(!object->getPoseAt(cloud->header.stamp, object_centroid, object_stamp))
			{
				ROS_ERROR("Object does not have previous poses, this should not happen.");
				return;
			}

			if(fabs((object_stamp - cloud->header.stamp).toSec()) > 1.0)
			{
				ROS_WARN("Delta time between recognition and tracking in the stale list: %f", (object_stamp - cloud->header.stamp).toSec());
			}

			float distance = pcl::euclideanDistance(
					current_recognition_centroid, object_centroid);
			if (distance < min_stale_distance)
			{
				min_stale_distance = distance;
				min_stale_ptr = object;
			}
		}

		//
		// If found remove it from the list, store it aside (out of the for loop) and mark the current recognition as solved
		//
		if (min_regular_distance <= min_new_distance
				&& min_regular_distance <= min_stale_distance)
		{
			// might have found something or not, need to check
			if (!min_regular_ptr)
			{
				ROS_INFO("Object %zu not found in the current lists.", rec_idx);
				continue;
			}

			recognition_solved[rec_idx] = true;
			object_for_recognition[rec_idx] = min_regular_ptr;
			regular_objects_list_copy.remove(min_regular_ptr);
			ROS_INFO("Object %zu found in the regular list.", rec_idx);
		}
		else if (min_new_distance < min_regular_distance
				&& min_new_distance < min_stale_distance)
		{
			recognition_solved[rec_idx] = true;
			object_for_recognition[rec_idx] = min_new_ptr;
			new_objects_list_copy.remove(min_new_ptr);
			ROS_INFO("Object %zu found in the new list. Promoting it.", rec_idx);
		}
		else if (min_stale_distance < min_new_distance
				&& min_stale_distance < min_regular_distance)
		{
			recognition_solved[rec_idx] = true;
			object_for_recognition[rec_idx] = min_stale_ptr;
			stale_objects_list_copy.remove(min_stale_ptr);
			ROS_INFO("Object %zu found in the stale list. Putting it back into the regular list.", rec_idx);
		}
		else
		{
			ROS_ERROR("Wait, wut?");
		}
	}

	// Increase the age for each object in the new list, if needed delete old objs
	// Same for stale objs, plus move all objects currently in the regular list into the stale list
	// Regular list should be empty, add back solved recognitions trackers

	// Work on the new list
	std::list<TrackedObjectPtr>::iterator list_iterator =
			new_objects_list_copy.begin();
	while (list_iterator != new_objects_list_copy.end())
	{
		TrackedObject& object = **list_iterator;

		if (object.newCount() > new_threshold_)
		{
			// discard the object, spurious recognition
			list_iterator = new_objects_list_copy.erase(list_iterator);
		}
		else
		{
			object.age()++;
			object.newCount()++;
			list_iterator++;
		}
	}

	// Work on the stale list
	list_iterator = stale_objects_list_copy.begin();
	while (list_iterator != stale_objects_list_copy.end())
	{
		TrackedObject& object = **list_iterator;

		if (object.staleCount() > stale_threshold_)
		{
			// discard the object, was tracking something else
			list_iterator = stale_objects_list_copy.erase(list_iterator);
		}
		else
		{
			object.age()++;
			object.staleCount()++;
			list_iterator++;
		}
	}

	// Empty the regular list
	list_iterator = regular_objects_list_copy.begin();
	while (list_iterator != regular_objects_list_copy.end())
	{
		TrackedObject& object = **list_iterator;

		object.staleCount() = 1;
		object.age()++;

		// Add the object to the stale list
		stale_objects_list_copy.push_back(*list_iterator);

		list_iterator = regular_objects_list_copy.erase(list_iterator);
	}

	assert(regular_objects_list_copy.empty());

	// Now add back the solved recognitions
	bool unsolved_recognitions = false;
	for (size_t i = 0; i < msg_recognition->objects.size(); ++i)
	{
		if (recognition_solved[i])
		{
			regular_objects_list_copy.push_back(object_for_recognition[i]);
		}
		else
		{
			unsolved_recognitions = true;
		}
	}

	//
	// If there are detections not considered yet then...
	//
	if (unsolved_recognitions)
	{

		// Check if the recognition result contains segmented clouds
		const bool segmentation_needed = msg_recognition->objects[0].point_clouds.empty();
		//TODO ....

		if (segmentation_needed)
		{
			//
			// Segment cloud, extract clusters
			//
			PointCloud::Ptr cloud_passthrough(new PointCloud());
			PointCloud::Ptr non_plane_cloud(new PointCloud());

//			// First, pass through
//			Utils::filterPassThrough(cloud, *cloud_passthrough);
//
//			// Second, find and remove plane
//			pcl::ModelCoefficients::Ptr coefficients(
//					new pcl::ModelCoefficients());
//			pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
//			Utils::planeSegmentation(cloud_passthrough, *coefficients,
//					*inliers);
//			if (inliers->indices.size() > 3)
//			{
//				PointCloud::Ptr cloud_projected(new PointCloud());
//				PointCloud::Ptr cloud_hull(new PointCloud());
//				std::vector<pcl::Vertices> hull_vertices;
//
//				Utils::planeProjection(cloud_passthrough, coefficients, inliers,
//						*cloud_projected);
//				Utils::convexHull(cloud_projected, *cloud_hull, hull_vertices);	//TODO
//				Utils::extractNonPlanePoints(cloud_passthrough, cloud_hull,
//						*non_plane_cloud);
//			}
//			else
//			{
//				ROS_WARN("Cannot segment plane.");
//				non_plane_cloud = cloud_passthrough;
//			}

			Utils::organizedMultiplaneSegmentation(cloud, *non_plane_cloud);

			cloud_no_plane_publisher_.publish(non_plane_cloud);

			// Third, cluster the cloud
			std::vector<pcl::PointIndices> cluster_indices;
			Utils::euclideanSegment(non_plane_cloud, cluster_indices);
			ROS_INFO("Extracted %zu segments.", cluster_indices.size());

			//
			// For each non solved recognition
			//

			// Assigning each recognition to the nearest cluster (still greedy)
			std::vector<bool> cluster_assigned(cluster_indices.size(), false);
			std::vector<pcl::PointXYZ> cluster_centroid(cluster_indices.size());

			// Compute the centroid for each cluster
			for (size_t i = 0; i < cluster_indices.size(); ++i)
			{
				Eigen::Vector4f centroid;
				pcl::compute3DCentroid(*non_plane_cloud, cluster_indices[i],
						centroid);
				cluster_centroid[i].getVector4fMap() = centroid;
			}

			for (size_t recognition_idx = 0;
					recognition_idx < msg_recognition->objects.size();
					++recognition_idx)
			{
				if (recognition_solved[recognition_idx])
				{
					continue;
				}

				pcl::PointXYZ recognition_centroid(
						msg_recognition->objects[recognition_idx].pose.pose.pose.position.x,
						msg_recognition->objects[recognition_idx].pose.pose.pose.position.y,
						msg_recognition->objects[recognition_idx].pose.pose.pose.position.z);

				// Find nearest cluster within threshold
				float nearest_cluster_distance = same_object_threshold_;
				int nearest_cluster_idx = -1;
				for (int i = 0; i < cluster_indices.size(); ++i)
				{
					if (cluster_assigned[i])
					{
						continue;
					}

					const float cluster_distance = pcl::euclideanDistance(
							recognition_centroid, cluster_centroid[i]);

					if (cluster_distance < nearest_cluster_distance)
					{
						nearest_cluster_distance = cluster_distance;
						nearest_cluster_idx = i;
					}
				}

				if (nearest_cluster_idx == -1)
				{
					// WARN: detection without a cluster
					ROS_WARN("Unable to find cluster for the recognition n. %zu.", recognition_idx);
					continue;
				}

				// assign cluster
				cluster_assigned[nearest_cluster_idx] = true;

				PointCloud::Ptr cluster_cloud(new PointCloud());
				Utils::extractCluster(non_plane_cloud,
						cluster_indices[nearest_cluster_idx], *cluster_cloud);

				// Init tracker
				TrackedObject::ParticleFilterTrackerPtr tracker;
				initTracker(cluster_cloud, tracker);
				std::stringstream ss;
				ss << msg_recognition->objects[recognition_idx].id.id << last_id_++;

				TrackedObjectPtr new_object(
						new TrackedObject(
								msg_recognition->objects[recognition_idx].id, ss.str(),
								tracker));

				// First step of tracking with the old cloud to initialize everything
				new_object->performTracking(downsampled_cloud);

				//
				// Add tracker to the new list
				//
				new_objects_list_copy.push_back(new_object);
				recognition_solved[recognition_idx] = true;
				ROS_INFO("Object %zu added to the new list.", recognition_idx);

			}
		}
		else // segmentation_needed
		{
			ROS_INFO(
					"Using cloud contained into the recognition array to init tracker");
			for (size_t recognition_idx = 0;
					recognition_idx < msg_recognition->objects.size();
					++recognition_idx)
			{
				if (recognition_solved[recognition_idx])
				{
					continue;
				}

				PointCloud::Ptr cluster_cloud(new PointCloud());
				pcl::fromROSMsg(
						msg_recognition->objects[recognition_idx].point_clouds[0],
						*cluster_cloud);

				//TODO shall I dpownsample?

				// Init tracker
				TrackedObject::ParticleFilterTrackerPtr tracker;
				initTracker(cluster_cloud, tracker);

				std::stringstream ss;
				ss << msg_recognition->objects[recognition_idx].id.id << last_id_++;

				TrackedObjectPtr new_object(
						new TrackedObject(
								msg_recognition->objects[recognition_idx].id, ss.str(),
								tracker));

				// First step of tracking with the old cloud to initialize everything
				new_object->performTracking(downsampled_cloud);

				//
				// Add tracker to the new list
				//
				new_objects_list_copy.push_back(new_object);
				recognition_solved[recognition_idx] = true;
				ROS_INFO("Object %zu added to the new list.", recognition_idx);
			}
		}

	} // unsolved_recognitions

	//
	// That's it, should be done, publish poses (regular and stale or only regular or add some "fresh" stale)
	//

	// Check if all the recognition have been solved
	for (size_t i = 0; i < msg_recognition->objects.size(); ++i)
	{
		if (!recognition_solved[i])
		{
			// WARNING
			const object_recognition_msgs::RecognizedObject& obj =
					msg_recognition->objects[i];
			ROS_WARN(
					"Unable to find a cluster for the recognition %zu with this centroid: %f %f %f", i, obj.pose.pose.pose.position.x, obj.pose.pose.pose.position.y, obj.pose.pose.pose.position.z);
		}
	}

	//
	// Publish regular and stale
	//
	publishPoses(cloud->header.frame_id, cloud->header.stamp, new_objects_list_copy, regular_objects_list_copy,
			stale_objects_list_copy);

	{
		boost::mutex::scoped_lock lists_lock(lists_mutex_);
		new_objects_list_ = new_objects_list_copy;
		regular_objects_list_ = regular_objects_list_copy;
		stale_objects_list_ = stale_objects_list_copy;
	}

//	/// OLD....
//	if (past_poses_.empty())
//	{
//		//just add them
//		for (int i = 0; i < msg_recognition->objects.size(); ++i)
//		{
//			past_poses_.insert(
//					std::pair<int, object_recognition_msgs::RecognizedObject>(i,
//							msg_recognition->objects[i]));
//		}
//	}
//	else
//	{
//		//TODO do something...
//	}
//
//	if (object_pose_publisher_.getNumSubscribers() > 0)
//	{
//		geometry_msgs::PoseArray pose_array;
//		pose_array.header.frame_id = frame_id_;
//		pose_array.header.stamp = ros::Time::now();
//
//		for (std::map<int, object_recognition_msgs::RecognizedObject>::iterator recognition_it =
//				past_poses_.begin(); recognition_it != past_poses_.end();
//				++recognition_it)
//		{
//			pose_array.poses.push_back(recognition_it->second.pose.pose.pose);
//		}
//
//		object_pose_publisher_.publish(pose_array);
//	}
}

void ObjectTrackerNode::cloudCallback(const PointCloud::ConstPtr& msg_cloud)
{
	{
		boost::mutex::scoped_lock prev_cloud_lock(previous_cloud_mutex_);
		if (!previous_cloud_)
		{
			// First time!
			frame_id_ = msg_cloud->header.frame_id;
		}
		previous_cloud_ = msg_cloud;
	}

	if((ros::Time::now() - msg_cloud->header.stamp).toSec() > 1.0)
	{
		ROS_WARN("Cloud callback running slow. Delta T > 1.0s: %f. Returning.", (ros::Time::now() - msg_cloud->header.stamp).toSec());
		return;
	}

	// Copy lists into local scope
	std::list<TrackedObjectPtr> new_objects_list_copy;
	std::list<TrackedObjectPtr> regular_objects_list_copy;
	std::list<TrackedObjectPtr> stale_objects_list_copy;
	{
		boost::mutex::scoped_lock lists_lock(lists_mutex_);
		new_objects_list_copy = new_objects_list_;
		regular_objects_list_copy = regular_objects_list_;
		stale_objects_list_copy = stale_objects_list_;
	}

	// Downsample
	PointCloud::Ptr non_plane_cloud(new PointCloud());
	PointCloud::Ptr cloud(new PointCloud());
	//Utils::organizedMultiplaneSegmentation(msg_cloud, *non_plane_cloud);
	Utils::gridSampleApprox(msg_cloud, *cloud);

	// setSearchMethod protected???
	pcl::search::KdTree<PointType>::Ptr search (new pcl::search::KdTree<PointType>());
	search->setInputCloud(cloud);

	//
	// For each tracker in each list:
	//
	// Perform tracking
	BOOST_FOREACH(TrackedObjectPtr& object, new_objects_list_copy)
	{
		object->performTracking(cloud, search);
	}

	BOOST_FOREACH(TrackedObjectPtr& object, regular_objects_list_copy)
	{
		object->performTracking(cloud, search);
	}

	BOOST_FOREACH(TrackedObjectPtr& object, stale_objects_list_copy)
	{
		object->performTracking(cloud, search);
	}

	// Publish poses
	// TODO Potential issue, publishing stale or deleted stuff
	publishPoses(msg_cloud->header.frame_id, msg_cloud->header.stamp, new_objects_list_copy, regular_objects_list_copy,
			stale_objects_list_copy);

	// OLD
	// do tracking using past_poses_
	//segment_tracker_.cloud_cb(msg_cloud);
}

geometry_msgs::Pose ObjectTrackerNode::getPoseFromObject(
		const TrackedObject& object) const
{
	geometry_msgs::Pose pose;

	const StateType& centroid = object.getCentroid();
	pose.position.x = centroid.x;
	pose.position.y = centroid.y;
	pose.position.z = centroid.z;

//	const Eigen::Quaternionf orientation(centroid.toEigenMatrix().rotation());
//	pose.orientation.w = orientation.w();
//	pose.orientation.x = orientation.x();
//	pose.orientation.y = orientation.y();
//	pose.orientation.z = orientation.z();

	pose.orientation.w = 1;
	pose.orientation.x = 0;
	pose.orientation.y = 0;
	pose.orientation.z = 0;

	return pose;
}

void ObjectTrackerNode::initTracker(const PointCloud::ConstPtr& object_cluster,
		TrackedObject::ParticleFilterTrackerPtr& tracker) const
{
	std::vector<double> default_step_covariance = std::vector<double>(6,
			0.015 * 0.015);
	default_step_covariance[3] *= 40.0;
	default_step_covariance[4] *= 40.0;
	default_step_covariance[5] *= 40.0;

	std::vector<double> initial_noise_covariance = std::vector<double>(6,
			0.00001);
	std::vector<double> default_initial_mean = std::vector<double>(6, 0.0);
	if (false)
	{
		// OMP
		boost::shared_ptr<
				pcl::tracking::ParticleFilterTracker<PointType, StateType> > specialized_track(
				new pcl::tracking::ParticleFilterOMPTracker<PointType, StateType>(
						2));
//		// Non OMP
//		boost::shared_ptr<
//				pcl::tracking::ParticleFilterTracker<PointType, StateType> > specialized_track(
//				new pcl::tracking::ParticleFilterTracker<PointType, StateType>());

		tracker = specialized_track;
	}
	else
	{
		// OMP
		boost::shared_ptr<
				pcl::tracking::KLDAdaptiveParticleFilterTracker<PointType,
						StateType> > specialized_track(
				new pcl::tracking::KLDAdaptiveParticleFilterOMPTracker<
						PointType, StateType>(2));
//		// NON OMP
//		boost::shared_ptr<
//		pcl::tracking::KLDAdaptiveParticleFilterTracker<PointType, StateType> > specialized_track(
//				new pcl::tracking::KLDAdaptiveParticleFilterTracker<PointType, StateType>());

		specialized_track->setMaximumParticleNum(650);
		specialized_track->setDelta(0.99);
		specialized_track->setEpsilon(0.2);
		StateType bin_size;
		bin_size.x = 0.05f;
		bin_size.y = 0.05f;
		bin_size.z = 0.05f;
		bin_size.roll = 0.1f;
		bin_size.pitch = 0.1f;
		bin_size.yaw = 0.1f;
		specialized_track->setBinSize(bin_size);
		tracker = specialized_track;
	}

	tracker->setTrans(Eigen::Affine3f::Identity());
	tracker->setStepNoiseCovariance(default_step_covariance);
	tracker->setInitialNoiseCovariance(initial_noise_covariance);
	tracker->setInitialNoiseMean(default_initial_mean);
	tracker->setIterationNum(1);

	tracker->setParticleNum(350);
	tracker->setResampleLikelihoodThr(0.00);
	tracker->setUseNormal(false);
	// setup coherences
	pcl::tracking::ApproxNearestPairPointCloudCoherence<PointType>::Ptr coherence =
			ApproxNearestPairPointCloudCoherence<PointType>::Ptr(
					new ApproxNearestPairPointCloudCoherence<PointType>());
	// pcl::tracking::NearestPairPointCloudCoherence<PointType>::Ptr coherence = pcl::tracking::NearestPairPointCloudCoherence<PointType>::Ptr
	//   (new pcl::tracking::NearestPairPointCloudCoherence<PointType> ());

	boost::shared_ptr<pcl::tracking::DistanceCoherence<PointType> > distance_coherence =
			boost::shared_ptr<pcl::tracking::DistanceCoherence<PointType> >(
					new pcl::tracking::DistanceCoherence<PointType>());

	coherence->addPointCoherence(distance_coherence);

	boost::shared_ptr<pcl::tracking::HSVColorCoherence<PointType> > color_coherence =
			boost::shared_ptr<pcl::tracking::HSVColorCoherence<PointType> >(
					new pcl::tracking::HSVColorCoherence<PointType>());
	color_coherence->setWeight(0.2);
	coherence->addPointCoherence(color_coherence);

	//boost::shared_ptr<pcl::search::KdTree<RefPointType> > search (new pcl::search::KdTree<RefPointType> (false));
//	boost::shared_ptr<pcl::search::Octree<PointType> > search(
//			new pcl::search::Octree<PointType>(0.01)); //TODO readd octree when linker is ok with that
	//boost::shared_ptr<pcl::search::OrganizedNeighbor<RefPointType> > search (new pcl::search::OrganizedNeighbor<RefPointType>);
//	coherence->setSearchMethod(search);
	coherence->setMaximumDistance(0.035);
	tracker->setCloudCoherence(coherence);

	//
	// Add cluster as a reference
	//
	PointCloud::Ptr ref_cloud(new PointCloud());
	PointCloud::Ptr centered_ref_cloud(new PointCloud());
	PointCloud::Ptr centered_ref_cloud_downsampled(new PointCloud());

	Utils::removeZeroPoints(object_cluster, *ref_cloud);

	Eigen::Vector4f object_centroid;

	PCL_INFO("calculating cog\n");

	pcl::compute3DCentroid<PointType>(*ref_cloud, object_centroid);

	// Transformation that brings the centroid at the origin
	Eigen::Affine3f transformation = Eigen::Affine3f::Identity();
	transformation.translation().matrix() = object_centroid.block<3, 1>(0, 0);

	pcl::transformPointCloud(*ref_cloud, *centered_ref_cloud,
			transformation.inverse());

	Utils::gridSample(centered_ref_cloud, *centered_ref_cloud_downsampled);
	tracker->setReferenceCloud(centered_ref_cloud_downsampled);
	tracker->setTrans(transformation);

	tracker->setMinIndices(int(centered_ref_cloud_downsampled->points.size()) / 2);
}

void ObjectTrackerNode::publishPoses(const std::string& frame_id, const ros::Time& stamp,
		const std::list<TrackedObjectPtr>& new_objects_list,
		const std::list<TrackedObjectPtr>& regular_objects_list,
		const std::list<TrackedObjectPtr>& stale_objects_list) const
{
	if (object_pose_publisher_.getNumSubscribers() == 0 && recognized_object_array_publisher_.getNumSubscribers() == 0 && object_names_publisher_.getNumSubscribers() == 0)
	{
		return;
	}

//	StateCloud particles_cloud;
//	particles_cloud.header.stamp = stamp;
//	particles_cloud.header.frame_id = frame_id;

	geometry_msgs::PoseArray pose_array;
	pose_array.header.stamp = stamp;
	pose_array.header.frame_id = frame_id;

	object_recognition_msgs::RecognizedObjectArray tracked_objects;
	tracked_objects.header.stamp = stamp;
	tracked_objects.header.frame_id = frame_id;

	visualization_msgs::MarkerArray object_names;
	std::string ns = nh_.getNamespace();
	int current_id = 0;

	// Poses for the regular objects
	std::list<TrackedObjectPtr>::const_iterator iter =
			regular_objects_list.begin();
	while (iter != regular_objects_list.end())
	{
		const TrackedObject& object = **iter;

		geometry_msgs::Pose pose = getPoseFromObject(object);
		pose_array.poses.push_back(pose);

//		particles_cloud += *(object.getParticles());

		object_recognition_msgs::RecognizedObject rec_obj;
		rec_obj.header.stamp = stamp;
		rec_obj.header.frame_id = frame_id;
		rec_obj.id = object.getObjectId();
		rec_obj.id.id = object.getName();
		rec_obj.pose.pose.pose = pose;
		tracked_objects.objects.push_back(rec_obj);

		visualization_msgs::Marker marker;
		marker.header.stamp = stamp;
		marker.header.frame_id = frame_id;
		marker.ns = ns;
		marker.id = current_id++;
		marker.frame_locked = true;
		marker.lifetime = ros::Duration(2,0);
		marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
		marker.text = object.getName();
		marker.pose  = pose;
		marker.scale.z = 0.1;
		marker.color.a = 1.0;
		marker.color.r = 1.0;
		marker.color.g = 1.0;
		marker.color.b = 1.0;
		object_names.markers.push_back(marker);

		iter++;
	}

	// Poses for the stale objects
	iter = stale_objects_list.begin();
	while (iter != stale_objects_list.end())
	{
		const TrackedObject& object = **iter;

		geometry_msgs::Pose pose = getPoseFromObject(object);
		pose_array.poses.push_back(pose);

//		particles_cloud += *(object.getParticles());

		object_recognition_msgs::RecognizedObject rec_obj;
		rec_obj.header.stamp = stamp;
		rec_obj.header.frame_id = frame_id;
		rec_obj.id = object.getObjectId();
		rec_obj.id.id = object.getName();
		rec_obj.pose.pose.pose = pose;
		tracked_objects.objects.push_back(rec_obj);

		visualization_msgs::Marker marker;
		marker.header.stamp = stamp;
		marker.header.frame_id = frame_id;
		marker.ns = ns;
		marker.id = current_id++;
		marker.frame_locked = true;
		marker.lifetime = ros::Duration(2,0);
		marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
		marker.text = object.getName();
		marker.pose  = pose;
		marker.scale.z = 0.1;
		marker.color.a = 1.0;
		marker.color.r = 1.0;
		marker.color.g = 1.0;
		marker.color.b = 1.0;
		object_names.markers.push_back(marker);

		iter++;
	}

//	// Poses for the new objects
//	iter = new_objects_list.begin();
//	while (iter != new_objects_list.end())
//	{
//		const TrackedObject& object = **iter;
//
//		geometry_msgs::Pose pose = getPoseFromObject(object);
//		pose_array.poses.push_back(pose);
//
////		particles_cloud += *(object.getParticles());
//
//		object_recognition_msgs::RecognizedObject rec_obj;
//		rec_obj.header.stamp = stamp;
//		rec_obj.header.frame_id = frame_id;
//		rec_obj.id = object.getObjectId();
//		rec_obj.id.id = object.getName();
//		rec_obj.pose.pose.pose = pose;
//		tracked_objects.objects.push_back(rec_obj);
//
//		visualization_msgs::Marker marker;
//		marker.header.stamp = stamp;
//		marker.header.frame_id = frame_id;
//		marker.ns = ns;
//		marker.id = current_id++;
//		marker.frame_locked = true;
//		marker.lifetime = ros::Duration(2,0);
//		marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
//		marker.text = object.getName();
//		marker.pose  = pose;
//		marker.scale.z = 0.1;
//		marker.color.a = 1.0;
//		marker.color.r = 1.0;
//		marker.color.g = 1.0;
//		marker.color.b = 1.0;
//		object_names.markers.push_back(marker);
//
//		iter++;
//	}

	// Publish
	if(object_pose_publisher_.getNumSubscribers() > 0)
		object_pose_publisher_.publish(pose_array);
	if(recognized_object_array_publisher_.getNumSubscribers() > 0)
		recognized_object_array_publisher_.publish(tracked_objects);
	if(object_names_publisher_.getNumSubscribers() > 0)
		object_names_publisher_.publish(object_names);

//	particles_publisher_.publish(particles_cloud);
}
