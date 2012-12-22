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

#ifndef OBJECTTRACKER_HPP_
#define OBJECTTRACKER_HPP_

#include <list>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <object_recognition_msgs/RecognizedObjectArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/tracking/tracking.h>

#include "PclSegmentTracker.hpp"
#include "TrackedObject.hpp"
#include "PointCloudUtils.hpp"

class ObjectTrackerNode
{
public:
	typedef pcl::PointXYZRGB PointType;
	typedef pcl::PointCloud<PointType> PointCloud;
	typedef pcl::tracking::ParticleXYZRPY StateType;
	typedef pcl::PointCloud<StateType> StateCloud;
	//TODO wrap all into namespace
	typedef ::TrackedObject<PointType, StateType> TrackedObject;
	typedef TrackedObject::Ptr TrackedObjectPtr;

	bool init(ros::CallbackQueueInterface *object_recognition_queue);

	ObjectTrackerNode(float same_object_threshold = 0.2, int new_threshold = 3,
			int stale_threshold = 1) :
			nh_object_recognition_(), nh_(), object_recognition_subscriber_(
					nh_object_recognition_, "recognized_object_array", 1), object_recognition_cloud_subscriber_(
					nh_object_recognition_, "cloud_in", 1), synchronizer_(
					SyncPolicy(15), object_recognition_subscriber_,
					object_recognition_cloud_subscriber_), cloud_subscriber_(), object_pose_publisher_(), last_id_(0),/*segment_tracker_(
			 "XXX", 2, 0.005, true, false, true, false),*/new_threshold_(
					new_threshold), stale_threshold_(stale_threshold), same_object_threshold_(
					same_object_threshold)
	{}

	void recognizedObjectCallback(
			const object_recognition_msgs::RecognizedObjectArrayConstPtr& msg_recognition, const PointCloud::ConstPtr& cloud);

	void cloudCallback(const PointCloud::ConstPtr& msg_cloud);

private:
	typedef PointCloudUtils<PointType> Utils;
	typedef message_filters::sync_policies::ApproximateTime<object_recognition_msgs::RecognizedObjectArray, PointCloud> SyncPolicy;

	// The nodeHandles
	ros::NodeHandle nh_object_recognition_;
	ros::NodeHandle nh_;
	// The object recognition result subscriber
	message_filters::Subscriber<object_recognition_msgs::RecognizedObjectArray> object_recognition_subscriber_;
	message_filters::Subscriber<PointCloud> object_recognition_cloud_subscriber_;
	message_filters::Synchronizer<SyncPolicy> synchronizer_;
	// The subscriber to the pointcloud
	ros::Subscriber cloud_subscriber_;
	// The pose publisher
	ros::Publisher object_pose_publisher_;
	ros::Publisher cloud_no_plane_publisher_;
	ros::Publisher particles_publisher_;
	ros::Publisher recognized_object_array_publisher_;
	ros::Publisher object_names_publisher_;

	// The frame id
	std::string frame_id_;
	// The last used id for a recognized object
	unsigned int last_id_;

	// PCL Segment tracker
	//OpenNISegmentTracking<PointType> segment_tracker_;

	// The last received cloud & its mutex
	boost::mutex previous_cloud_mutex_;
	PointCloud::ConstPtr previous_cloud_;

	// Object lists & mutex
	boost::mutex lists_mutex_;
	std::list<TrackedObjectPtr> new_objects_list_;
	std::list<TrackedObjectPtr> regular_objects_list_;
	std::list<TrackedObjectPtr> stale_objects_list_;

	// Thresholds
	const int new_threshold_;
	const int stale_threshold_;
	const float same_object_threshold_;

	//
	// Functions
	//
	geometry_msgs::Pose getPoseFromObject(const TrackedObject& object) const;
	void initTracker(const PointCloud::ConstPtr& object_cluster,
			TrackedObject::ParticleFilterTrackerPtr& tracker) const;
	void publishPoses(const std::string& frame_id, const ros::Time& stamp, const std::list<TrackedObjectPtr>& new_objects_list,
			const std::list<TrackedObjectPtr>& regular_objects_list,
			const std::list<TrackedObjectPtr>& stale_objects_list) const;

};

#endif //OBJECTTRACKER_HPP_
