#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Tommaso Cavallari

import sys
import time
import math
import threading
import rospy
import actionlib
import numpy as np
import scipy
import tf
from numpy import linalg
from dynamic_reconfigure.server import Server
from object_recognition_msgs.msg import RecognizedObjectArray, RecognizedObject, ObjectId, ObjectRecognitionAction, ObjectRecognitionGoal, ObjectRecognitionResult
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Point, Vector3, PoseArray, Pose, PointStamped
from std_msgs.msg import Header 
from visualization_msgs.msg import MarkerArray, Marker
from object_tracker.cfg import ObjectTrackerConfig
from copy import copy, deepcopy
from object_tracker import TrackedObject, MotionParameters, BaseTracker 
from object_tracker.tracker_factory import create_tracker

class ObjectTracker:
    _initialized = False
    _model_valid = False
    _progressive_id = 0
    
    _tracker = BaseTracker
    
    _ork_camera_frame = ""
    _base_tf_frame = ""
    
    _dynamic_reconfigure_server = Server
    _object_detection_client = actionlib.SimpleActionClient
    _tf_publisher = tf.TransformBroadcaster
    _tf_listener = tf.TransformListener
    _marker_publisher = rospy.Publisher
    
    _detection_rate = 0.0
    _tf_rate = 0.0
    _detection_timer = None
    _tf_timer = None
    
    _model_lock = threading.Lock
    _detection_lock = threading.Lock
    _tf_lock = threading.Lock
    
    _tracked_objects = set()
    
    _static_object_threshold = 0.0
    _static_object_window = 0.0
    _min_poses_for_estimation = 0
    _min_poses_to_consider_an_object = 0
    _max_stale_time_for_object = 0.0
    _same_object_threshold = 0.0    
    _use_roi = False
    _roi_limits = []
    
    # TODO now the ids are not considering the DB field, fix that
    def __init__(self):
        self._initialized = False
        self._model_valid = False
        self._model_lock = threading.Lock()
        self._detection_lock = threading.Lock()
        self._tf_lock = threading.Lock()

        self._tracked_objects = set()
        self._static_object_threshold = 0.025
        self._static_object_window = 4
        self._max_stale_time_for_object = 10.0
        self._min_poses_to_consider_an_object = 5
        self._min_poses_for_estimation = 10
        self._progressive_id = 0
        self._same_object_threshold = 0.1
        self._use_roi = False
        self._roi_limits = []
        self._detection_rate = 2.0
        self._tf_rate = 20.0
        self._ork_camera_frame = "" 
        
    def tf_frame_for_object(self, obj):
        """ Return a formatted string that uniquely identifies an object, based on its database and progressive id. """
        return "/tracked_object_%s_%s" % (obj.id, obj.progressive_id)
    
    def publish_markers(self):
        """ 
        Publish visualization markers for each tracked object.

        The markers are frame locked with the object's TF frame hence moving continuously as long as the TF frame is updated. 
        """
        tracked_objs_copy = set()
        with self._model_lock:
            tracked_objs_copy = copy(self._tracked_objects)
        
        marker_array = MarkerArray()
        id = 0
        now = rospy.Time.now()
        for obj in tracked_objs_copy:
            if len(obj.poses) < self._min_poses_to_consider_an_object:
                continue
            marker = Marker()
            marker.header.stamp = now
            marker.header.frame_id = obj.recognized_object.header.frame_id
            marker.id = id
            marker.lifetime = rospy.Duration(10.0)
            id += 1
            marker.ns = "tracked_objects"
            marker.action = Marker.ADD
            marker.type = Marker.SPHERE
            marker.pose = obj.recognized_object.pose.pose.pose
            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker.frame_locked = True
            
            marker_array.markers.append(marker)
            
        self._marker_publisher.publish(marker_array) 
            
    def find_closest_object_from_list(self, object, object_list, max_dist = sys.float_info.max):
        """
        Find the closest object from a list using L2 distance.

        Args:
            object: the RecognizedObject to which the nearest neighbor has to be found
            object_list: a list of TrackedObject
            max_dist: the maximum distance allowed between the object and its closest neighbor

        Returns:
            the closest TrackedObject if found or None
        """
        if not object_list:
            return None
        
        closest_obj = None
        min_dist = max_dist
        
        for obj in object_list:
            distance = self.l2_dist(object.pose, obj.poses[-1])
            if distance < min_dist:
                closest_obj = obj
                min_dist = distance
        
        return closest_obj        
                
    def initialization_phase_behavior(self, data):
        """
        The behavior during initialization phase.

        Since a motion model has not yet been estimated, a simple tracking mechanism in which objects from different recognition results are
        considered the same object if they are close enough is used. 
        When a certain minimum number of poses for a single object have been found, the estimation
        of the motion model is attempted. If the estimation is successful the internal tracker state is set to initialized 
        and the subsequent behavior is different 
        (tracking_phase_behavior).

        Args:
            data: a RecognizedObjectArray containing the results of the object detection
        """
        # check if in the current recognition there are multiple instances of a single obj id
        categorized_detection_result = dict()
        
        tracked_objs_copy = set()
        with self._model_lock:
            tracked_objs_copy = copy(self._tracked_objects)
          
        # categorization based only on the id, not the db, fixme    
        for obj in data.objects:
            if obj.id.id in categorized_detection_result:
                categorized_detection_result[obj.id.id].append(obj)
            else:
                categorized_detection_result[obj.id.id] = [obj]
        
        for objects in categorized_detection_result.itervalues():
            if objects:
                potential_objs = []
                for tracked_obj in tracked_objs_copy:
                    if tracked_obj.id == objects[0].id.id and tracked_obj.db == objects[0].id.db:
                        potential_objs.append(tracked_obj)
                
                for object in objects:
                    # perform initialization
                            
                    if not potential_objs:
                        rospy.logdebug("Adding a obj with id %s" % object.id)
                        obj_to_append = TrackedObject()
                        obj_to_append.id = object.id.id
                        obj_to_append.db = object.id.db
                        obj_to_append.confidence = object.confidence
                        obj_to_append.poses = [ object.pose ]
                        obj_to_append.stamps = [ object.header.stamp ]
                        obj_to_append.recognized_object = deepcopy(object)
                        
                        tracked_objs_copy.add(obj_to_append)
                        # nothing else can be done for this obj...
                        continue
                    
                    closest_potential_obj = self.find_closest_object_from_list(object, potential_objs, self._same_object_threshold)
                    if closest_potential_obj is not None:
                        # add current pose to the tracked obj
                        # remove the closest object from the potential list
                        potential_objs.remove(closest_potential_obj)
                        closest_potential_obj.poses.append(object.pose)
                        
                        if len(closest_potential_obj.poses) > self._min_poses_for_estimation:
                            rospy.loginfo("Object %s [%s]: I have %d poses now. Estimating model." % (closest_potential_obj.id, closest_potential_obj.db, len(closest_potential_obj.poses)))
                            if self._tracker.init_model_from_object(closest_potential_obj):
                                # setup ids
                                closest_potential_obj.progressive_id = self._progressive_id
                                
                                #change the coordinates of the object according to the new reference frame
                                closest_potential_obj.recognized_object.header.frame_id = self.tf_frame_for_object(closest_potential_obj)
                                #closest_potential_obj.recognized_object.header.stamp = now
                                closest_potential_obj.recognized_object.pose.header = closest_potential_obj.recognized_object.header
                                closest_potential_obj.recognized_object.pose.pose.pose.position.x = 0.0
                                closest_potential_obj.recognized_object.pose.pose.pose.position.y = 0.0
                                closest_potential_obj.recognized_object.pose.pose.pose.position.z = 0.0
                                closest_potential_obj.recognized_object.pose.pose.pose.orientation.x = 0.0
                                closest_potential_obj.recognized_object.pose.pose.pose.orientation.y = 0.0
                                closest_potential_obj.recognized_object.pose.pose.pose.orientation.z = 0.0
                                closest_potential_obj.recognized_object.pose.pose.pose.orientation.w = 1.0
                                
                                self._progressive_id += 1
                                tracked_objs_copy.clear() 
                                tracked_objs_copy.add(closest_potential_obj)
                                                                   
                                with self._model_lock:
                                    self._tracked_objects = tracked_objs_copy 
                                    self._initialized = True 
                                    self._model_valid = True
                                # if the model has been initialized return now
                                return                                        
                
        with self._model_lock:
            self._tracked_objects = tracked_objs_copy
                
    def tracking_phase_behavior(self, data):
        """
        The behavior during the tracking phase.

        Each object is tracked considering its polar coordinates wrt. the center of rotation, the tracking is more accurate in this way.
        Also, after each tacking phase the rotation model is updated to reflect the increased number of data points.

        Args:
            data: a RecognizedObjectArray containing the results of the object detection
        """
        #standard tracking
        new_tracked_objects = set()
        
        tracked_objs_copy = set()
        with self._model_lock:
            tracked_objs_copy = copy(self._tracked_objects)
        
        for obj in data.objects:
            pose = PoseStamped()
            pose.header = obj.header
            pose.pose = obj.pose.pose.pose
            
            object_parameters = self._tracker.get_motion_parameters(pose)
            if object_parameters is None:
                continue
            
            closest_obj = None
            min_dist = self._same_object_threshold
            for tracked_obj in tracked_objs_copy:
                if tracked_obj.id == obj.id.id and tracked_obj.db == obj.id.db:
                    obj_dist = self._tracker.compute_distance(object_parameters, tracked_obj.motion_parameters)
                    l2_dist = self.l2_dist(obj.pose, tracked_obj.poses[-1])

                    rospy.logdebug("Obj Dist == %s ; L2 Dist = %s" % (obj_dist, l2_dist))
                    if obj_dist < min_dist:
                        min_dist = obj_dist
                        closest_obj = tracked_obj
                    #elif l2_dist < min_dist:
                    #  min_dist = l2_dist
                    #  closest_obj = tracked_obj
                        
            if closest_obj is not None:
                # add the current pose
                closest_obj.poses.append(obj.pose)
                closest_obj.stamps.append(obj.header.stamp)
                # remove the object from the tracking set
                # put it into the new set
                new_tracked_objects.add(closest_obj)
                tracked_objs_copy.remove(closest_obj)
            else:
                # create a new object to track
                # add it to the new set
                tracked_object = TrackedObject()
                tracked_object.id = obj.id.id
                tracked_object.db = obj.id.db
                tracked_object.confidence = obj.confidence
                tracked_object.progressive_id = self._progressive_id
                tracked_object.motion_parameters = object_parameters
                tracked_object.poses = [obj.pose]
                tracked_object.stamps = [obj.header.stamp]
                tracked_object.recognized_object = deepcopy(obj)
                
                # change coordinates according to the rotation frame
                tracked_object.recognized_object.header.frame_id = self.tf_frame_for_object(tracked_object)
                #tracked_object.recognized_object.header.stamp = now
                tracked_object.recognized_object.pose.header = tracked_object.recognized_object.header
                tracked_object.recognized_object.pose.pose.pose.position.x = 0.0
                tracked_object.recognized_object.pose.pose.pose.position.y = 0.0
                tracked_object.recognized_object.pose.pose.pose.position.z = 0.0
                tracked_object.recognized_object.pose.pose.pose.orientation.x = 0.0
                tracked_object.recognized_object.pose.pose.pose.orientation.y = 0.0
                tracked_object.recognized_object.pose.pose.pose.orientation.z = 0.0
                tracked_object.recognized_object.pose.pose.pose.orientation.w = 1.0
                
#                 if there is already an object in that position don't add the new one 
                if (self._tracker.find_closest_object(object_parameters, tracked_objs_copy, self._same_object_threshold) is None and
                        self._tracker.find_closest_object(object_parameters, new_tracked_objects, self._same_object_threshold) is None):                    
                    new_tracked_objects.add(tracked_object)
                    self._progressive_id += 1
                else:
                    rospy.logdebug("Skipping object insertion for object %s" % tracked_object.id)                    
                     
        # add back the new list to the old list
        tracked_objs_copy |= new_tracked_objects
        
        with self._model_lock:
            self._tracked_objects = tracked_objs_copy
        
        # update motion model...  
        self._tracker.update_model(data.header, tracked_objs_copy)
        
        if self._marker_publisher.get_num_connections() > 0:
            self.publish_markers()
            
        self._tracker.publish_messages(tracked_objs_copy)
            
    def remove_static_objects(self):
        """
        Remove static objects from the tracked objects set.

        An object is declared static if in the last user-configurable number of poses it has moved less than a certain distance.
        """
        objs_to_remove = set()
        tracked_objs_copy = set()
        with self._model_lock:
            tracked_objs_copy = copy(self._tracked_objects)
            
        for obj in tracked_objs_copy:
            if len(obj.poses) < self._static_object_window:
                continue
        
            obj_movement = 0.0
            prev_pose = self.pose_to_array(obj.poses[-self._static_object_window])   
            for pose_idx in range(len(obj.poses) - self._static_object_window, len(obj.poses)):
                pose = self.pose_to_array(obj.poses[pose_idx]) 
                obj_movement += np.linalg.norm(pose - prev_pose)    
                prev_pose = pose
               
            if obj_movement < self._static_object_threshold:
                rospy.logdebug("Removing %s at position %s since it's not moving." % (obj.id, prev_pose))
                objs_to_remove.add(obj)
            
        with self._model_lock:
            self._tracked_objects -= objs_to_remove                       
    
        
    def recognized_object_callback(self, data):
        """
        The callback for the object recognition.

        The main algorithm, behaving as a finite state machine.
        1 - Objects older than a configurable amount of time are removed from the tracked objects set.
        2 - If necessary the input object poses are converted into an user specificable reference frame 
            (as an example to account for movements of the robot using the base_link frame)
        3 - Static objects are removed.
        4 - If the algorithm has lost track of each object a re-initialization is performed.
        5 - If not initialized perform the initialization_phase_behavior else perform the tracking_phase_behavior.

        Args:
            data: a RecognizedObjectArray containing the object detection results
        """
        #remove old objects
        with self._model_lock:
            time = rospy.Time.now().to_sec()
            self._tracked_objects = set(x for x in self._tracked_objects if (time - x.poses[-1].header.stamp.to_sec() < self._max_stale_time_for_object))
            
        if self._ork_camera_frame != data.header.frame_id:
            self._ork_camera_frame = data.header.frame_id
        
        if not data.objects:
            return
        
        if self._base_tf_frame == "":
            self._base_tf_frame = data.header.frame_id
        elif self._base_tf_frame != data.header.frame_id:
            # Transform all poses into the correct RF
            for obj in data.objects:
                pose = PoseStamped()
                pose.header = obj.header
                pose.pose = obj.pose.pose.pose
                try:
                    pose = self._tf_listener.transformPose(self._base_tf_frame, pose)
                    obj.header = pose.header
                    obj.pose.header = pose.header
                    obj.pose.pose.pose = pose.pose
                except tf.Exception, e:
                    rospy.logerr(e)
                    return
                
        # remove static objects from the tracking set
        self.remove_static_objects()
        
        # if no objects are tracked re init the model, but keep publishing the old tf frames...
        if self._initialized:
            with self._model_lock:
                reinit = True
                for obj in self._tracked_objects:
                    if len(obj.poses) > self._min_poses_to_consider_an_object:
                        reinit = False
                        break
                    
                if reinit:
                    rospy.logdebug("Lost track of every object, re-initializing....")
                    self._initialized = False
                
        if not self._initialized:
            self.initialization_phase_behavior(data)
        else:
            self.tracking_phase_behavior(data)
            
    def pose_to_array(self, pose):
        """ Convert a PoseWithCovarianceStamped into a numpy.array. """
        return np.array([ pose.pose.pose.position.x, 
                          pose.pose.pose.position.y, 
                          pose.pose.pose.position.z ])
    
    def l2_dist(self, pose1, pose2):
        """ Compute the Euclidean distance between two PoseWithCovarianceStamped. """
        return np.linalg.norm(self.pose_to_array(pose1) - self.pose_to_array(pose2))
                              
    def tf_callback(self, event):
        """ A callback used by a timer to publish TF data. """
        with self._tf_lock:
            if not self._model_valid:
                return
            
            with self._model_lock:
                self._tracker.broadcast_tf(event.current_real, self._tracked_objects)
                
    def transform_roi_limits(self):
        """ 
        Transform the limits of the user specified Region of Interest for the object detection 
        from an user specified reference_frame to the camera reference frame (used by ORK). 
        """
        # Has to be done for all the 8 cube points...
        # The resulting cube is guaranteed to contain the original cube
        header = Header()
        header.frame_id = self._base_tf_frame
        header.stamp = rospy.Time(0)
        
        points = np.zeros(shape=(8,4))
                    
        try:
            transformation = self._tf_listener.asMatrix(self._ork_camera_frame, header)
            
            point = np.array([ self._roi_limits[0], self._roi_limits[2], self._roi_limits[4], 1.0 ])      
            points[0,:] = np.dot(transformation, point)
            
            point = np.array([ self._roi_limits[0], self._roi_limits[3], self._roi_limits[4], 1.0 ])           
            points[1,:] = np.dot(transformation, point)
            
            point = np.array([ self._roi_limits[0], self._roi_limits[2], self._roi_limits[5], 1.0 ])           
            points[2,:] = np.dot(transformation, point)
            
            point = np.array([ self._roi_limits[0], self._roi_limits[3], self._roi_limits[5], 1.0 ])           
            points[3,:] = np.dot(transformation, point)
            
            point = np.array([ self._roi_limits[1], self._roi_limits[2], self._roi_limits[4], 1.0 ])           
            points[4,:] = np.dot(transformation, point)
            
            point = np.array([ self._roi_limits[1], self._roi_limits[3], self._roi_limits[4], 1.0 ])           
            points[5,:] = np.dot(transformation, point)
            
            point = np.array([ self._roi_limits[1], self._roi_limits[2], self._roi_limits[5], 1.0 ])           
            points[6,:] = np.dot(transformation, point)
            
            point = np.array([ self._roi_limits[1], self._roi_limits[3], self._roi_limits[5], 1.0 ])           
            points[7,:] = np.dot(transformation, point)                                             
            
            return [ min(points[:,0]), 
                     max(points[:,0]), 
                     min(points[:,1]), 
                     max(points[:,1]),
                     min(points[:,2]), 
                     max(points[:,2])]
            
        except tf.Exception, e:
            rospy.logerr("Error while transforming ROI limits: %s" % e)
            return self._roi_limits
            
    def detection_timer_callback(self, event):
        """ A callback invoked at an user specified rate that performs the object recognition task and model estimation. """
        with self._detection_lock:
            goal = ObjectRecognitionGoal()
            goal.use_roi = self._use_roi
            if self._use_roi:
#                if False: # look into the roi transofrmation
                if self._base_tf_frame != "" and self._ork_camera_frame != "" and self._base_tf_frame != self._ork_camera_frame:
                    # transform the limits into the camera frame (since it's the only frame ORK understands)
                    rospy.logdebug("Limits before: %s" % self._roi_limits)
                    goal.filter_limits = self.transform_roi_limits()
                    rospy.logdebug("Limits after: %s" % goal.filter_limits)
                else:
                    goal.filter_limits = self._roi_limits
            
            start_time = rospy.Time.now()
            self._object_detection_client.send_goal_and_wait(goal, rospy.Duration(5.0))
            if self._object_detection_client.get_state() == actionlib.GoalStatus.SUCCEEDED:
                self.recognized_object_callback(self._object_detection_client.get_result().recognized_objects)
                
            rospy.logdebug("The detection took %s and returned %s." % ((rospy.Time.now() - start_time).to_sec(), self._object_detection_client.get_state()))
            
    def set_parameters(self, config):   
        """
        Set the object tracking parameters.

        Args:
            config: a dictionary containing the parameters to be set. Look into RotatingObjectTracker.cfg to see which parameters are allowed.
        """     
        # frames
        if not (self._base_tf_frame == config['fixed_frame']):
            self._base_tf_frame = config['fixed_frame']
            self.reinit_model()
        
        # tracking params
        self._min_poses_to_consider_an_object = config['min_poses_for_tracking']
        self._max_poses_for_object = config['max_poses_for_object']
        self._max_stale_time_for_object = config['max_stale_time']
        self._same_object_threshold = config ['same_object_threshold']
        self._use_roi = config['use_roi']
        self._roi_limits = [ config['x_min'], config['x_max'],
                             config['y_min'], config['y_max'],
                             config['z_min'], config['z_max'] ]        
        self._static_object_window = config['static_object_detection_window']
        self._static_object_threshold = config['static_object_threshold']
        
        # rates
        if self._detection_rate != config['detection_rate']:
            self._detection_rate = config['detection_rate']
            if self._detection_timer is not None:
                self._detection_timer.shutdown()
            self._detection_timer = rospy.Timer(rospy.Duration(1.0 / self._detection_rate), self.detection_timer_callback)
            
        if self._tf_rate != config['tf_rate']:
            self._tf_rate = config['tf_rate']
            if self._tf_timer is not None:
                self._tf_timer.shutdown()
            self._tf_timer = rospy.Timer(rospy.Duration(1.0 / self._tf_rate), self.tf_callback)
    
    def dynamic_reconfigure_callback(self, config, level):   
        """ A callback for the dynamic_reconfigure server. """   
        self.set_parameters(config)
        self._tracker.set_parameters(config)            
        return config
    
    def reinit_model(self):
        """ Reinitializes the tracker clearing the tracked objects list. """
        with self._model_lock:
            self._tracked_objects.clear()
        self._initialized = False
        self._progressive_id = 0
    
    def start(self, tracker_instance):
        """ Start the object tracker. """
        rospy.init_node("object_tracker")
        
        # Setup tracker
        self._tracker = tracker_instance
        
        # dynamic reconfigure params
        self._dynamic_reconfigure_server = Server(ObjectTrackerConfig, self.dynamic_reconfigure_callback)
        
        # params from launch file 
        self.set_parameters(rospy.get_param("~"))        
        
        # Publishers
        self._marker_publisher = rospy.Publisher("objects_markers", MarkerArray)
        
        # Object recognition server
        rospy.loginfo("Waiting for object recognition server...")
        self._object_detection_client = actionlib.SimpleActionClient("recognize_objects", ObjectRecognitionAction)
        self._object_detection_client.wait_for_server()
        self._detection_timer = rospy.Timer(rospy.Duration(1.0 / self._detection_rate), self.detection_timer_callback)
        
        # setup the tf publisher
        self._tf_publisher = tf.TransformBroadcaster()
        self._tf_timer = rospy.Timer(rospy.Duration(1.0 / self._tf_rate), self.tf_callback)
        
        self._tf_listener = tf.TransformListener()
        
        self._tracker.start(self._tf_publisher, self._tf_listener)
        rospy.loginfo("Object Tracker started")
        
        rospy.spin()
    
if __name__ == "__main__":
    motion_tracker = create_tracker()
    tracker = ObjectTracker()
    tracker.start(motion_tracker)
    
