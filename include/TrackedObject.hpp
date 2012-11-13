/*
 * TrackedObject.hpp
 *
 *  Created on: Nov 6, 2012
 *      Author: tcavallari
 */

#ifndef TRACKEDOBJECT_HPP_
#define TRACKEDOBJECT_HPP_

#include <deque>
#include <ros/ros.h>
#include <object_recognition_msgs/ObjectId.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/tracking/tracking.h>
#include <pcl/tracking/particle_filter.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>

template<typename PointType = pcl::PointXYZRGB,
		typename StateType = pcl::tracking::ParticleXYZRPY>
class TrackedObject
{
public:
	typedef pcl::tracking::ParticleFilterTracker<PointType, StateType> ParticleFilterTracker;

	// not good because ParticleFilterTracker inherits Tracker's Ptr that doesn't have getResult public
	//typedef typename pcl::tracking::ParticleFilterTracker<PointType, StateType>::Ptr ParticleFilterTrackerPtr;
	typedef boost::shared_ptr<
			pcl::tracking::ParticleFilterTracker<PointType, StateType> > ParticleFilterTrackerPtr;

	typedef boost::shared_ptr<TrackedObject<PointType, StateType> > Ptr;
	typedef boost::shared_ptr<const TrackedObject<PointType, StateType> > ConstPtr;

	typedef std::pair<ros::Time, StateType> TimeState;

	typedef pcl::PointCloud<PointType> PointCloud;
	typedef typename PointCloud::ConstPtr PointCloudConstPtr;
	typedef pcl::PointCloud<StateType> StateCloud;
	typedef typename StateCloud::ConstPtr StateCloudConstPtr;

	TrackedObject(const object_recognition_msgs::ObjectId& object_id, const std::string& name,
			ParticleFilterTrackerPtr& tracker) :
			object_id_(object_id), name_ (name), tracker_(tracker), regular_count_(0), new_count_(
					0), stale_count_(0), age_(0)
	{
	}

	inline object_recognition_msgs::ObjectId getObjectId() const
	{
		return object_id_;
	}

//	inline ParticleFilterTrackerPtr getTracker() const
//	{
//		return tracker_;
//	}

	inline StateType getCentroid() const
	{
		return tracker_->getResult();
	}

	inline StateCloudConstPtr getParticles() const
	{
		return tracker_->getParticles();
	}

	inline int age() const
	{
		return age_;
	}

	inline int& age()
	{
		return age_;
	}

	inline int regularCount() const
	{
		return regular_count_;
	}

	inline int& regularCount()
	{
		return regular_count_;
	}

	inline int newCount() const
	{
		return new_count_;
	}

	inline int& newCount()
	{
		return new_count_;
	}

	inline int staleCount() const
	{
		return stale_count_;
	}

	inline int& staleCount()
	{
		return stale_count_;
	}

	inline std::deque<TimeState> getPastPoses()
	{
		return past_poses_;
	}

	inline std::string getName() const
	{
		return name_;
	}

	void performTracking(const PointCloudConstPtr& cloud,
			const typename pcl::search::Search<PointType>::Ptr& search)
	{
		tracker_->setInputCloud(cloud);
		tracker_->setSearchMethod(search);
		tracker_->compute();

		{
			//boost::mutex::scoped_lock past_poses_lock(past_poses_mutex_);
			past_poses_.push_back(
					TimeState(cloud->header.stamp, tracker_->getResult()));
		}
	}

	void performTracking(const PointCloudConstPtr& cloud)
	{
		typename pcl::search::KdTree<PointType>::Ptr search(
				new pcl::search::KdTree<PointType>());
		search->setInputCloud(cloud);
		performTracking(cloud, search);
	}

	bool getPoseAt(const ros::Time& time, StateType& pose, ros::Time& stamp)
	{
		std::deque<TimeState> past_poses_copy;

		{
		//	boost::mutex::scoped_lock past_poses_lock(past_poses_mutex_);
			past_poses_copy = past_poses_;
		}

		if (past_poses_copy.empty())
			return false;

		TimeState result = *past_poses_copy.begin();
		if (result.first <= time)
		{
			typename std::deque<TimeState>::const_iterator iter =
					past_poses_copy.begin()++;
			while(iter != past_poses_copy.end() && iter->first <= time)
			{
				iter++;
			}
			if (iter != past_poses_copy.end())
			{
				// check which time is the nearest
				double delta_after = fabs(iter->first.toSec() - time.toSec());
				double delta_before = fabs(
						(iter - 1)->first.toSec() - time.toSec());
				if (delta_before < delta_after)
				{
					result = *(--iter);
				}
				else
				{
					result = *iter;
				}
			}
			else
			{
				result = *(--iter);
			}
		}

		pose = result.second;
		stamp = result.first;
		return true;
	}

private:
	// The objectId
	const object_recognition_msgs::ObjectId object_id_;
	// The object name (unique)
	const std::string name_;
	// The tracker
	const ParticleFilterTrackerPtr tracker_;
	// A status/age variable
	int regular_count_;
	int new_count_;
	int stale_count_;
	int age_;

	boost::mutex past_poses_mutex_;
	std::deque<TimeState> past_poses_;

};

inline bool operator==(const object_recognition_msgs::ObjectId& a,
		const object_recognition_msgs::ObjectId& b)
{
	return (a.db == b.db) && (a.id == b.id);
}

inline bool operator!=(const object_recognition_msgs::ObjectId& a,
		const object_recognition_msgs::ObjectId& b)
{
	return !(a == b);
}

#endif /* TRACKEDOBJECT_HPP_ */
