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
		//tracker_->setSearchMethod(search);
		tracker_->compute();

		{
			boost::mutex::scoped_lock past_poses_lock(past_poses_mutex_);
			past_poses_.push_front(
					TimeState(cloud->header.stamp, tracker_->getResult()));
			if(past_poses_.size() > 30)
				past_poses_.resize(30);
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
			boost::mutex::scoped_lock past_poses_lock(past_poses_mutex_);
			past_poses_copy = past_poses_;
		}

		if (past_poses_copy.empty())
			return false;

		TimeState result = *(past_poses_copy.rbegin());
		if (result.first <= time)
		{
			typename std::deque<TimeState>::const_reverse_iterator iter =
					past_poses_copy.rbegin()++;
			while(iter != past_poses_copy.rend() && iter->first <= time)
			{
				iter++;
			}
			if (iter != past_poses_copy.rend())
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
