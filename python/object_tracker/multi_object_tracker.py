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
from object_tracker.srv import EstimateRotation, EstimateRotationResponse, EstimateRotationRequest
from object_tracker.msg import RotationParameters
from object_tracker.cfg import RotatingObjectTrackerConfig
from copy import copy

class TrackedObject:
    id = 0
    db = ""
    progressive_id = 0
    poses = []
    stamps = [] 
    radius = 0.0
    phase = 0.0
    confidence = 0.0   

class Tracker:
    _initialized = False
    _model_valid = False
    _progressive_id = 0
    
    _ork_camera_frame = ""
    _recognized_objects_topic = "/recognized_object_array"
    _base_tf_frame = ""
    _intermediate_tf_frame = "/rotation_center"
    _rotating_tf_frame = "/rotating_objects"
    
    _dynamic_reconfigure_server = Server
    _estimate_rotation_service = EstimateRotation
    _object_detection_client = actionlib.SimpleActionClient
    _tf_publisher = tf.TransformBroadcaster
    _tf_listener = tf.TransformListener
    _marker_publisher = rospy.Publisher
    _rotation_publisher = rospy.Publisher
    _rotating_objects_publisher = rospy.Publisher
    
    _detection_rate = 0.0
    _tf_rate = 0.0
    _detection_timer = None
    _tf_timer = None
    
    _model_lock = threading.Lock
    _detection_lock = threading.Lock
    _tf_lock = threading.Lock
    
    _reference_frame = np.array
    _rotation_center = np.array
    _rotation_axis = np.array
    _rotation_speed = np.array
    _previous_angle = 0.0
    _last_tf_broadcast = 0.0   
    
    _tracked_objects = set()
    
    _static_object_threshold = 0.0
    _static_object_window = 0.0
    _min_poses_for_estimation = 0
    _min_poses_to_consider_an_object = 0
    _max_poses_for_object = 0
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
        self._rotation_center = np.zeros(3)
        self._rotation_axis = np.zeros(3)
        self._rotation_speed = np.zeros(1)
        self._previous_angle = 0.0
        self._last_tf_broadcast = 0.0   
        self._reference_frame = np.identity(4)
        self._tracked_objects = set()
        self._static_object_threshold = 0.025
        self._static_object_window = 4
        self._max_poses_for_object = 50
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
        
#    def init_all_objects(self):
#        for obj in self._tracked_objects:
#            avg_radius = 0.0
#            for pose in obj.poses:
#                point = np.array([pose.pose.pose.position.x, pose.pose.pose.position.y, pose.pose.pose.position.z])
#                avg_radius += linalg.norm(point - self._rotation_center)
#            avg_radius /= len(obj.poses)
#            obj.radius = avg_radius 
#
#            last_pose = np.array([obj.poses[-1].pose.pose.position.x, obj.poses[-1].pose.pose.position.y, obj.poses[-1].pose.pose.position.z])
#            last_pose -= self._rotation_center
#            last_pose = np.matrix(self._reference_frame[:3,:3]).I.dot(last_pose)
#     
##            print "lp", last_pose
##            print "radius", obj.radius
##            print "phase", math.atan2(last_pose[0,1], last_pose[0,0])
#                            
#        pass

    def tf_frame_for_object(self, obj):
        return "%s_%s_%s" % (self._rotating_tf_frame, obj.id, obj.progressive_id)
    
    def broadcast_tf(self, time):
        # the lock on the model should have been acquired outside
        self._previous_angle = self._rotation_speed[-1] * (time.to_sec() - self._last_tf_broadcast) + self._previous_angle
        self._tf_publisher.sendTransform(self._rotation_center[-1,:], tf.transformations.quaternion_from_matrix(self._reference_frame), 
                                         time, self._intermediate_tf_frame, self._base_tf_frame)
        self._tf_publisher.sendTransform([0.0, 0.0, 0.0], tf.transformations.quaternion_about_axis(self._previous_angle, [0.0, 0.0, 1.0]),
                                         time, self._rotating_tf_frame, self._intermediate_tf_frame)
        
        for obj in self._tracked_objects:
            if len(obj.poses) < self._min_poses_to_consider_an_object:
                continue
            
            obj_x = obj.radius * math.cos(obj.phase)
            obj_y = obj.radius * math.sin(obj.phase)
            self._tf_publisher.sendTransform([obj_x, obj_y, 0.0], [0.0, 0.0, 0.0, 1.0], time, self.tf_frame_for_object(obj), self._rotating_tf_frame)
        
        self._last_tf_broadcast = time.to_sec()
    
    
    
    def publish_markers(self):
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
            marker.header.frame_id = self.tf_frame_for_object(obj)
            marker.id = id
            marker.lifetime = rospy.Duration(10.0)
            id += 1
            marker.ns = "rotating_objects"
            marker.action = Marker.ADD
            marker.type = Marker.SPHERE
            marker.pose.position.x = 0.0 #obj.radius * math.cos(obj.phase)
            marker.pose.position.y = 0.0 #obj.radius * math.sin(obj.phase)
            marker.pose.position.z = 0.0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
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
        
    def publish_rotation_msg(self, new_centers, new_axii, new_speeds):        
        rotation_msg = RotationParameters()
        rotation_msg.header.frame_id = self._base_tf_frame
        rotation_msg.header.stamp = rospy.Time.now()
        
        rotation_msg.center.x = self._rotation_center[-1, 0]
        rotation_msg.center.y = self._rotation_center[-1, 1]
        rotation_msg.center.z = self._rotation_center[-1, 2]
        
        if len(new_centers) > 1:
            rotation_msg.center_covariance = np.cov(new_centers, rowvar=0).flatten().tolist()
        else:
            rotation_msg.center_covariance = np.identity(3).flatten().tolist()
            
        if self._rotation_center.shape[0] > 1:
            rotation_msg.center_time_covariance = np.cov(self._rotation_center, rowvar=0).flatten().tolist()
        else:
            rotation_msg.center_time_covariance = np.identity(3).flatten().tolist()            
        
        rotation_msg.axis.x = self._rotation_axis[-1, 0]
        rotation_msg.axis.y = self._rotation_axis[-1, 1]
        rotation_msg.axis.z = self._rotation_axis[-1, 2]
        
        if len(new_axii) > 1:
            rotation_msg.axis_covariance = np.cov(new_axii, rowvar=0).flatten().tolist()
        else:
            rotation_msg.axis_covariance = np.identity(3).flatten().tolist()
            
        if self._rotation_axis.shape[0] > 1:
            rotation_msg.axis_time_covariance = np.cov(self._rotation_axis, rowvar=0).flatten().tolist()
        else:
            rotation_msg.axis_time_covariance = np.identity(3).flatten().tolist()       
        
        rotation_msg.speed = self._rotation_speed[-1]
        rotation_msg.speed_std_dev = np.std(self._rotation_speed)
        rotation_msg.speed_time_std_dev = np.std(self._rotation_speed)
        
        self._rotation_publisher.publish(rotation_msg)
      
    def publish_rotating_objects(self):
        recognized_objects = RecognizedObjectArray()
        tracked_objs_copy = set()
        with self._model_lock:
            tracked_objs_copy = copy(self._tracked_objects)
        
        now = rospy.Time().now()
        recognized_objects.header.frame_id = self._base_tf_frame
        recognized_objects.header.stamp = now
        for obj in tracked_objs_copy:
            if len(obj.poses) < self._min_poses_to_consider_an_object:
                continue
                        
            recognized_obj = RecognizedObject()
            recognized_obj.id.id = obj.id
            recognized_obj.id.db = obj.db
            recognized_obj.confidence = obj.confidence
            recognized_obj.header.frame_id = self.tf_frame_for_object(obj)
            recognized_obj.header.stamp = now
            recognized_obj.pose.header = recognized_obj.header
            recognized_obj.pose.pose.pose.position.x = 0.0
            recognized_obj.pose.pose.pose.position.y = 0.0
            recognized_obj.pose.pose.pose.position.z = 0.0
            recognized_obj.pose.pose.pose.orientation.x = 0.0
            recognized_obj.pose.pose.pose.orientation.y = 0.0
            recognized_obj.pose.pose.pose.orientation.z = 0.0
            recognized_obj.pose.pose.pose.orientation.w = 1.0
            
            recognized_objects.objects.append(recognized_obj)
        
        self._rotating_objects_publisher.publish(recognized_objects)          
    
    def update_model(self, header):
        num_models = 0
        new_axii = []
        new_centers = []
        new_speeds = []
        
        tracked_objs_copy = set()
        with self._model_lock:
            tracked_objs_copy = copy(self._tracked_objects)        
        
        for obj in tracked_objs_copy:
            if len(obj.poses) > self._min_poses_for_estimation:
                try:
                    request = EstimateRotationRequest()
                    request.poses = obj.poses
                    response = EstimateRotationResponse()
                    response = self._estimate_rotation_service(request)
                    if response.success:
                        new_axii.append(np.array([response.axis.x, response.axis.y, response.axis.z]))
                        new_centers.append(np.array([response.center.x, response.center.y, response.center.z]))
                        new_speeds.append(response.speed)
                        num_models += 1
#                        print "Response: ", response
                except rospy.ServiceException, e:
                    rospy.logerr("Error! %s" % e)
                    
            if len(obj.poses) > self._max_poses_for_object:
                del obj.poses[:-self._max_poses_for_object]
                  
        if num_models > 0:                    
            new_axis = np.mean(new_axii, axis=0)
            new_center = np.mean(new_centers, axis=0)
            new_speed = np.mean(new_speeds)                           
            
            if new_axis is not [0.0,1.0,0.0] and new_axis is not [0, -1, 0]:
                x_axis = np.cross(new_axis, [0.0,1.0,0.0])
            else:
                x_axis = np.cross(new_axis, [1.0,0.0,0.0])
                
            y_axis = np.cross(new_axis, x_axis)
            rot_matr =  np.array([[x_axis[0], y_axis[0], new_axis[0], 0.0], 
                                  [x_axis[1], y_axis[1], new_axis[1], 0.0],
                                  [x_axis[2], y_axis[2], new_axis[2], 0.0],
                                  [0.0,0.0,0.0,1.0]])
            
            with self._model_lock:
                self._rotation_center = np.vstack((self._rotation_center, new_center))
                if self._rotation_center.shape[0] > self._max_poses_for_object:
                    self._rotation_center = self._rotation_center[-self._max_poses_for_object:-1,:]
                    
                self._rotation_axis = np.vstack((self._rotation_axis, new_axis))
                if self._rotation_axis.shape[0] > self._max_poses_for_object:
                    self._rotation_axis = self._rotation_axis[-self._max_poses_for_object:-1,:]
                    
                self._rotation_speed = np.vstack((self._rotation_speed, new_speed))
                if self._rotation_speed.shape[0] > self._max_poses_for_object:
                    self._rotation_speed = self._rotation_speed[-self._max_poses_for_object:-1]               
                    
                self._reference_frame = rot_matr
                
                # now or at the time the detection was performed?
                self.broadcast_tf(rospy.Time.now())
#                self.broadcast_tf(header.stamp)
            
                if self._rotation_publisher.get_num_connections() > 0:
                    self.publish_rotation_msg(new_centers, new_axii, new_speeds) 
                
            rospy.logdebug("Updated model: center: %s axis: %s speed: %s" % (new_center, new_axis, new_speed))
            
            # Update already tracked objs to reflect the new model
            rospy.logdebug("There are %s tracked objects." % len(tracked_objs_copy))
            for obj in tracked_objs_copy:
                obj_pose = PoseStamped()
                obj_pose.header = obj.poses[-1].header
                obj_pose.pose = obj.poses[-1].pose.pose
                try: 
                    obj_pose = self._tf_listener.transformPose(self._rotating_tf_frame, obj_pose)
                    obj.radius = math.sqrt(obj_pose.pose.position.x**2 + obj_pose.pose.position.y**2)
                    obj.phase = math.atan2(obj_pose.pose.position.y, obj_pose.pose.position.x)
                except tf.Exception, e:
                    rospy.logwarn("%s" % e)
                    
    def init_model_from_object(self, object):
        request = EstimateRotationRequest()
        request.poses = object.poses
        response = EstimateRotationResponse()
        try:
            response = self._estimate_rotation_service(request)
        except rospy.ServiceException, e:
            rospy.logerr("Error! %s" % e)
            return False
        
        rospy.logdebug("Initialization: received a response: %s" % response)
        if response.success:
            object.radius = response.radius                
            # TODO phase
            
            z_axis = np.array([response.axis.x, response.axis.y, response.axis.z])
            if z_axis is not [0.0,1.0,0.0] and z_axis is not [0.0, -1.0, 0.0]:
                x_axis = np.cross(z_axis, [0.0,1.0,0.0])
            else:
                x_axis = np.cross(z_axis, [1.0,0.0,0.0])
                
            y_axis = np.cross(z_axis, x_axis)
            rot_matr =  np.array([[x_axis[0], y_axis[0], z_axis[0], 0.0], 
                                  [x_axis[1], y_axis[1], z_axis[1], 0.0],
                                  [x_axis[2], y_axis[2], z_axis[2], 0.0],
                                  [0.0,0.0,0.0,1.0]])
            with self._model_lock:
                self._rotation_center = np.array([[response.center.x, response.center.y, response.center.z]])
                self._rotation_axis = np.array([z_axis])
                self._rotation_speed = np.array([response.speed])
                self._reference_frame = rot_matr
                self._initialized = True 
                self._model_valid = True   
            
            rospy.loginfo("Initialization successful.")
            
        return response.success
    
    def find_closest_object_from_list(self, object, object_list, max_dist = sys.float_info.max):
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
    
    def find_closest_object_from_list_polar(self, object, object_list, max_dist = sys.float_info.max):
        if not object_list:
            return None
        
        closest_obj = None
        min_dist = max_dist
        
        for obj in object_list:
            distance = self.polar_dist(object.radius, object.phase, obj.radius, obj.phase)
            if distance < min_dist:
                closest_obj = obj
                min_dist = distance
        
        return closest_obj  
                
    def initialization_phase_behavior(self, data):
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
                            if self.init_model_from_object(closest_potential_obj):
                                # setup ids
                                closest_potential_obj.progressive_id = self._progressive_id
                                self._progressive_id += 1
                                tracked_objs_copy.clear() 
                                tracked_objs_copy.add(closest_potential_obj)                                   
                                with self._model_lock:
                                    self._tracked_objects = tracked_objs_copy 
                                # if the model got initialized return now
                                return                                        
#                else:
#                    rospy.loginfo("More than 1 object with id = %s, ambiguous initialization." % id)
#                    # TODO, add behavior
                
        with self._model_lock:
            self._tracked_objects = tracked_objs_copy
                
    def tracking_phase_behavior(self, data):
        #standard tracking
        new_tracked_objects = set()
        
        tracked_objs_copy = set()
        with self._model_lock:
            tracked_objs_copy = copy(self._tracked_objects)
        
        for obj in data.objects:
            pose = PoseStamped()
            pose.header = obj.header
            pose.pose = obj.pose.pose.pose
            
            try:
                pose = self._tf_listener.transformPose(self._rotating_tf_frame, pose)
            except tf.Exception, e:
                rospy.logwarn("Tf exception: %s" % e)
                continue
                            
            radius = math.sqrt(pose.pose.position.x**2 + pose.pose.position.y**2)
            phase = math.atan2(pose.pose.position.y, pose.pose.position.x)
            rospy.logdebug("Radius: %s Phase: %s" % (radius, phase))
            closest_obj = None
            min_dist = self._same_object_threshold #sys.float_info.max
            for tracked_obj in tracked_objs_copy:
                if tracked_obj.id == obj.id.id and tracked_obj.db == obj.id.db:
                    polar_dist = self.polar_dist(radius, phase, tracked_obj.radius, tracked_obj.phase)
                    l2_dist = self.l2_dist(obj.pose, tracked_obj.poses[-1])

                    rospy.logdebug("Polar Dist == %s ; L2 Dist = %s" % (polar_dist, l2_dist))
                    if polar_dist < min_dist:
                        min_dist = polar_dist
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
                self._progressive_id += 1
                tracked_object.phase = phase
                tracked_object.radius = radius
                tracked_object.poses = [obj.pose]
                tracked_object.stamps = [obj.header.stamp]
                
#                 if there is already an object in that position don't add the new one 
                if (self.find_closest_object_from_list_polar(tracked_object, tracked_objs_copy, self._same_object_threshold) is None and
                        self.find_closest_object_from_list_polar(tracked_object, new_tracked_objects, self._same_object_threshold) is None):                    
                    new_tracked_objects.add(tracked_object)
                else:
                    rospy.loginfo("Skipping object insertion for object %s" % tracked_object.id)                    
                     
        # add back the new list to the old list
        tracked_objs_copy |= new_tracked_objects
        
        with self._model_lock:
            self._tracked_objects = tracked_objs_copy
        
        # update motion model...  
        self.update_model(data.header)
        
        if self._marker_publisher.get_num_connections() > 0:
            self.publish_markers()
            
        if self._rotating_objects_publisher.get_num_connections() > 0:
            self.publish_rotating_objects()
            
    def remove_static_objects(self):
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
        return np.array([ pose.pose.pose.position.x, 
                          pose.pose.pose.position.y, 
                          pose.pose.pose.position.z ])
    
    def polar_dist(self, r1, phi1, r2, phi2):
        return math.sqrt(r1**2 + r2**2 - 2*r1*r2*math.cos(phi1 - phi2))
    
    def l2_dist(self, pose1, pose2):
        return np.linalg.norm(self.pose_to_array(pose1) - self.pose_to_array(pose2))
                              
    def tf_callback(self, event):
        with self._tf_lock:
            if not self._model_valid:
                return
            
            with self._model_lock:
                self.broadcast_tf(event.current_real)
                
    def transform_roi_limits(self):
        # Has to be done for all the 8 cube points...
        header = Header()
        header.frame_id = self._base_tf_frame
        header.stamp = rospy.Time(0)
        
        points = np.zeros(shape=(8,4))
                    
        try:
            transformation = self._tf_listener.asMatrix(self._ork_camera_frame, header)
            
            point = np.array([ self._roi_limits[0], self._roi_limits[2], self._roi_limits[4], 0.0 ])            
            points[0,:] = np.dot(transformation, point)
            
            point = np.array([ self._roi_limits[0], self._roi_limits[3], self._roi_limits[4], 0.0 ])            
            points[1,:] = np.dot(transformation, point)
            
            point = np.array([ self._roi_limits[0], self._roi_limits[2], self._roi_limits[5], 0.0 ])            
            points[2,:] = np.dot(transformation, point)
            
            point = np.array([ self._roi_limits[0], self._roi_limits[3], self._roi_limits[5], 0.0 ])            
            points[3,:] = np.dot(transformation, point)
            
            point = np.array([ self._roi_limits[1], self._roi_limits[2], self._roi_limits[4], 0.0 ])            
            points[4,:] = np.dot(transformation, point)
            
            point = np.array([ self._roi_limits[1], self._roi_limits[3], self._roi_limits[4], 0.0 ])            
            points[5,:] = np.dot(transformation, point)
            
            point = np.array([ self._roi_limits[1], self._roi_limits[2], self._roi_limits[5], 0.0 ])            
            points[6,:] = np.dot(transformation, point)
            
            point = np.array([ self._roi_limits[1], self._roi_limits[3], self._roi_limits[5], 0.0 ])            
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
        with self._detection_lock:
            goal = ObjectRecognitionGoal()
            goal.use_roi = self._use_roi
            if self._use_roi:
                if False: # look into the roi transofrmation
#                if self._base_tf_frame != "" and self._ork_camera_frame != "" and self._base_tf_frame != self._ork_camera_frame:
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
        # frames
        if not (self._base_tf_frame == config['fixed_frame'] 
                and self._intermediate_tf_frame == config['rotation_center_frame'] 
                and self._rotating_tf_frame == config['rotating_frame']):
            self._base_tf_frame = config['fixed_frame']
            self._intermediate_tf_frame = config['rotation_center_frame']
            self._rotating_tf_frame = config['rotating_frame']
            with self._model_lock:
                self._tracked_objects.clear()
            self._initialized = False
            self._progressive_id = 0
        
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
        self.set_parameters(config)
        return config

    
    def start(self):
        rospy.init_node("rotating_object_tracker")
        
        # dynamic reconfigure params
        self._dynamic_reconfigure_server = Server(RotatingObjectTrackerConfig, self.dynamic_reconfigure_callback)
        
        # params from launch file 
        self.set_parameters(rospy.get_param("~"))        
        
        # estimation service
        rospy.wait_for_service("estimate_rotation")
        self._estimate_rotation_service = rospy.ServiceProxy("estimate_rotation", EstimateRotation, True)
        
        # Publishers
        self._marker_publisher = rospy.Publisher("rotating_objects", MarkerArray)
        self._rotation_publisher = rospy.Publisher("rotation_parameters", RotationParameters)
        self._rotating_objects_publisher = rospy.Publisher("recognized_rotating_objects", RecognizedObjectArray)
        
        # Object recognition server
        rospy.loginfo("Waiting for object recognition server...")
        self._object_detection_client = actionlib.SimpleActionClient("recognize_objects", ObjectRecognitionAction)
        self._object_detection_client.wait_for_server()
        self._detection_timer = rospy.Timer(rospy.Duration(1.0 / self._detection_rate), self.detection_timer_callback)
        
        # setup the tf publisher
        self._tf_publisher = tf.TransformBroadcaster()
        self._tf_timer = rospy.Timer(rospy.Duration(1.0 / self._tf_rate), self.tf_callback)
        
        self._tf_listener = tf.TransformListener()
        rospy.loginfo("started")
        
        rospy.spin()
    
if __name__ == "__main__":
    tracker = Tracker()
    tracker.start()
    
