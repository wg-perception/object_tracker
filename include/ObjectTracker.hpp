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

	ObjectTrackerNode(float same_object_threshold = 0.25, int new_threshold = 2,
			int stale_threshold = 4) :
			nh_object_recognition_(), nh_(), object_recognition_subscriber_(
					nh_object_recognition_, "recognized_object_array", 1), object_recognition_cloud_subscriber_(
					nh_object_recognition_, "cloud_in", 1), synchronizer_(
					SyncPolicy(15), object_recognition_subscriber_,
					object_recognition_cloud_subscriber_), cloud_subscriber_(), object_pose_publisher_(), /*segment_tracker_(
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

	// The frame id
	std::string frame_id_;

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
