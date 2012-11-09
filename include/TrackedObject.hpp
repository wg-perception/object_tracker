/*
 * TrackedObject.hpp
 *
 *  Created on: Nov 6, 2012
 *      Author: tcavallari
 */

#ifndef TRACKEDOBJECT_HPP_
#define TRACKEDOBJECT_HPP_

#include <object_recognition_msgs/ObjectId.h>
#include <pcl/point_types.h>
#include <pcl/tracking/tracking.h>
#include <pcl/tracking/particle_filter.h>

template<typename PointType = pcl::PointXYZRGB,
		typename StateType = pcl::tracking::ParticleXYZRPY>
class TrackedObject {
public:
	typedef pcl::tracking::ParticleFilterTracker<PointType, StateType> ParticleFilterTracker;

	// not good because ParticleFilterTracker inherits Tracker's Ptr that doesn't have getResult public
	//typedef typename pcl::tracking::ParticleFilterTracker<PointType, StateType>::Ptr ParticleFilterTrackerPtr;
	typedef boost::shared_ptr<
			pcl::tracking::ParticleFilterTracker<PointType, StateType> > ParticleFilterTrackerPtr;

	typedef boost::shared_ptr<TrackedObject<PointType, StateType> > Ptr;
	typedef boost::shared_ptr<const TrackedObject<PointType, StateType> > ConstPtr;

	TrackedObject(const object_recognition_msgs::ObjectId& object_id,
			ParticleFilterTrackerPtr& tracker) :
			object_id_(object_id), tracker_(tracker), regular_count_(0), new_count_(
					0), stale_count_(0), age_(0) {
	}

	inline object_recognition_msgs::ObjectId getObjectId() const {
		return object_id_;
	}

	inline ParticleFilterTrackerPtr getTracker() const {
		return tracker_;
	}

	inline StateType getCentroid() const {
		return tracker_->getResult();
	}

	inline int age() const {
		return age_;
	}

	inline int& age() {
		return age_;
	}

	inline int regularCount() const {
		return regular_count_;
	}

	inline int& regularCount() {
		return regular_count_;
	}

	inline int newCount() const {
		return new_count_;
	}

	inline int& newCount() {
		return new_count_;
	}

	inline int staleCount() const {
		return stale_count_;
	}

	inline int& staleCount() {
		return stale_count_;
	}

private:
	// The objectId
	const object_recognition_msgs::ObjectId object_id_;
	// The tracker
	const ParticleFilterTrackerPtr tracker_;
	// A status/age variable
	int regular_count_;
	int new_count_;
	int stale_count_;
	int age_;

};

inline bool operator==(const object_recognition_msgs::ObjectId& a,
		const object_recognition_msgs::ObjectId& b) {
	return (a.db == b.db) && (a.id == b.id);
}

inline bool operator!=(const object_recognition_msgs::ObjectId& a,
		const object_recognition_msgs::ObjectId& b) {
	return !(a == b);
}

#endif /* TRACKEDOBJECT_HPP_ */
