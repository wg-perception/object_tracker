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
import numpy as np
import scipy
import tf
from numpy import linalg
from object_recognition_msgs.msg import RecognizedObjectArray, RecognizedObject, ObjectId
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Point, Vector3, PoseArray, Pose, PointStamped
from visualization_msgs.msg import MarkerArray, Marker
from object_tracker.srv import EstimateRotation, EstimateRotationResponse, EstimateRotationRequest
from copy import copy

class TrackedObject:
    id = 0
    poses = []
    stamps = [] 
    radius = 0.0
    phase = 0.0   

class Tracker:
    _initialized = False
    _recognized_objects_topic = "/recognized_object_array"
    _base_tf_frame = "/camera_rgb_optical_frame"
    _intermediate_tf_frame = "/rotation_center"
    _rotating_tf_frame = "/rotating_objects"
    _tf_publisher = tf.TransformBroadcaster
    _tf_listener = tf.TransformListener
    _marker_publisher = rospy.Publisher
    _model_lock = threading.Lock
    _reference_frame = np.array
    _rotation_center = np.array
    _rotation_axis = np.array
    _rotation_speed = 0
    _previous_angle = 0
    _last_tf_broadcast = 0.0   
    _tracked_objects = []
    _estimate_rotation_service = EstimateRotation
    _max_poses_for_object = 0
    
    # TODO now the ids are not considering the DB field, fix that
    def __init__(self):
        self._initialized = False
        self._model_lock = threading.Lock()
        self._rotation_center = np.zeros(3)
        self._rotation_axis = np.zeros(3)
        self._rotation_speed = 0.0
        self._previous_angle = 0
        self._last_tf_broadcast = 0.0   
        self._reference_frame = np.identity(4)
        self._tracked_objects = []
        self._max_poses_for_object = 50
        
    def init_all_objects(self):
        for obj in self._tracked_objects:
            avg_radius = 0.0
            for pose in obj.poses:
                point = np.array([pose.pose.pose.position.x, pose.pose.pose.position.y, pose.pose.pose.position.z])
                avg_radius += linalg.norm(point - self._rotation_center)
            avg_radius /= len(obj.poses)
            obj.radius = avg_radius 

            last_pose = np.array([obj.poses[-1].pose.pose.position.x, obj.poses[-1].pose.pose.position.y, obj.poses[-1].pose.pose.position.z])
            last_pose -= self._rotation_center
            last_pose = np.matrix(self._reference_frame[:3,:3]).I.dot(last_pose)
     
            print "lp", last_pose
            print "radius", obj.radius
            print "phase", math.atan2(last_pose[0,1], last_pose[0,0])
                            
        pass
    
    def publish_markers(self):
        marker_array = MarkerArray()
        id = 0
        now = rospy.Time.now()
        for obj in self._tracked_objects:
            if len(obj.poses) < 3:
                continue
            marker = Marker()
            marker.header.stamp = now
            marker.header.frame_id = self._rotating_tf_frame
            marker.id = id
            marker.lifetime = rospy.Duration(10)
            id += 1
            marker.ns = "rotating_objects"
            marker.action = Marker.ADD
            marker.type = Marker.SPHERE
            marker.pose.position.x = obj.radius * math.cos(obj.phase)
            marker.pose.position.y = obj.radius * math.sin(obj.phase)
            marker.pose.position.z = 0.0
            marker.pose.orientation.x = 0
            marker.pose.orientation.y = 0
            marker.pose.orientation.z = 0
            marker.pose.orientation.w = 1
            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            marker.color.r = 1.0
            marker.color.g = 0
            marker.color.b = 0
            marker.color.a = 1.0
            marker.frame_locked = True
            
            marker_array.markers.append(marker)
            
        self._marker_publisher.publish(marker_array)

    
    def update_model(self):
        num_models = 0
        new_axis = np.array([0,0,0])
        new_center = np.array([0,0,0])
        new_speed = 0.0
        
        
        for obj in self._tracked_objects:
            if len(obj.poses) > 10:
                try:
                    request = EstimateRotationRequest()
                    request.poses = obj.poses
                    response = EstimateRotationResponse()
                    response = self._estimate_rotation_service(request)
                    if response.success:
                        new_axis = new_axis + np.array([response.axis.x, response.axis.y, response.axis.z])
                        new_center = new_center + np.array([response.center.x, response.center.y, response.center.z])
                        new_speed += response.theta
                        num_models += 1
                        print "Response: ", response
                except rospy.ServiceException, e:
                    rospy.logerr("Error! %s" % e)
                    
            if len(obj.poses) > self._max_poses_for_object:
                del obj.poses[:-self._max_poses_for_object]
                 
        print "Pre update model: center: %s axis: %s speed: %s" % (new_center, new_axis, new_speed) 
        if num_models > 0:                    
            new_axis /= num_models
            new_center /= num_models
            new_speed /= num_models
            
            if new_axis is not [0,1,0] and new_axis is not [0, -1, 0]:
                x_axis = np.cross(new_axis, [0,1,0])
            else:
                x_axis = np.cross(new_axis, [1,0,0])
                
            y_axis = np.cross(new_axis, x_axis)
            rot_matr =  np.array([[x_axis[0], y_axis[0], new_axis[0], 0], 
                                  [x_axis[1], y_axis[1], new_axis[1], 0],
                                  [x_axis[2], y_axis[2], new_axis[2], 0],
                                  [0,0,0,1]])
            
            with self._model_lock:
                self._rotation_center = new_center
                self._rotation_axis = new_axis
                self._rotation_speed = new_speed
                self._reference_frame = rot_matr
                
                time = rospy.Time.now()
                self._previous_angle = self._rotation_speed * (time.to_sec() - self._last_tf_broadcast) + self._previous_angle
                self._tf_publisher.sendTransform(self._rotation_center, tf.transformations.quaternion_from_matrix(self._reference_frame), 
                                                 time, self._intermediate_tf_frame, self._base_tf_frame)
                self._tf_publisher.sendTransform([0, 0, 0], tf.transformations.quaternion_about_axis(self._previous_angle, [0, 0, 1]),
                                                 time, self._rotating_tf_frame, self._intermediate_tf_frame)
                self._last_tf_broadcast = time.to_sec()
                
            print "Updated model: center: %s axis: %s speed: %s" % (new_center, new_axis, new_speed)
            
            #todo update tracked objs (radius phi)
            print "There are %s tracked objects." % len(self._tracked_objects)
            for obj in self._tracked_objects:
                obj_pose = PoseStamped()
                obj_pose.header = obj.poses[-1].header
                obj_pose.pose = obj.poses[-1].pose.pose
                
                obj_pose = self._tf_listener.transformPose(self._rotating_tf_frame, obj_pose)
                obj.radius = math.sqrt(obj_pose.pose.position.x**2 + obj_pose.pose.position.y**2)
                obj.phase = math.atan2(obj_pose.pose.position.y, obj_pose.pose.position.x)
                
                
    
    def recognized_object_callback(self, data):
        if not self._initialized:
            
            # check if in the current recognition there are multiple instances of a single obj id
            categorized_detection_result = dict()
            for obj in data.objects:
                if obj.id.id in categorized_detection_result:
                    categorized_detection_result[obj.id.id].append(obj)
                else:
                    categorized_detection_result[obj.id.id] = [obj]
            
            for id, objects in categorized_detection_result.iteritems():
                if len(objects) == 1:
                    # perform initialization
                    potential_objs = []
                    for tracked_obj in self._tracked_objects:
                        if tracked_obj.id == objects[0].id.id:
                            potential_objs.append(tracked_obj)
                            
                    if not potential_objs:
                        obj_to_append = TrackedObject()
                        obj_to_append.id = objects[0].id.id
                        obj_to_append.poses = [objects[0].pose]
                        obj_to_append.stamps = [objects[0].header.stamp]
                        
                        self._tracked_objects.append(obj_to_append)
                        # nothing else can be done for this obj...
                        continue
                    
                    if len(potential_objs) == 1:
                        # add current pose to the tracked obj
                        potential_objs[0].poses.append(objects[0].pose)
                        if len(potential_objs[0].poses) > 10:
                            #TODO estimate model, be done with initialization....
                            rospy.loginfo("Object %s: I have %d poses now. calling service." % (potential_objs[0].id, len(potential_objs[0].poses)))
                            request = EstimateRotationRequest()
                            request.poses = potential_objs[0].poses
                            try:
                                response = EstimateRotationResponse()
                                response = self._estimate_rotation_service(request)
                                rospy.loginfo("Initialization: received a response: %s" % response)
                                if response.success:
                                    potential_objs[0].radius = response.radius
                                    # TODO phase
                                    z_axis = np.array([response.axis.x, response.axis.y, response.axis.z])
                                    if z_axis is not [0,1,0] and z_axis is not [0, -1, 0]:
                                        x_axis = np.cross(z_axis, [0,1,0])
                                    else:
                                        x_axis = np.cross(z_axis, [1,0,0])
                                        
                                    y_axis = np.cross(z_axis, x_axis)
                                    rot_matr =  np.array([[x_axis[0], y_axis[0], z_axis[0], 0], 
                                                          [x_axis[1], y_axis[1], z_axis[1], 0],
                                                          [x_axis[2], y_axis[2], z_axis[2], 0],
                                                          [0,0,0,1]])
                                    with self._model_lock:
                                        self._rotation_center = np.array([response.center.x, response.center.y, response.center.z])
                                        self._rotation_axis = z_axis
                                        self._rotation_speed = response.theta
                                        self._reference_frame = rot_matr
                                        self._initialized = True    
                                    
                                    rospy.loginfo("Initialization successful.")
#                                    self.init_all_objects()
                                    self._tracked_objects = []                                    
                                    return # if the model got initialized return                                        
                            except rospy.ServiceException, e:
                                rospy.logerr("Error! %s" % e)
                                
                        continue
                            
                    # potential_objs contains more than one match
                    # TODO: nearest one, RANSAC?
                    rospy.loginfo("More than 1 object already tracked with id = %s, still TODO." % id)                        
                else:
                    rospy.loginfo("More than 1 object with id = %s, ambiguous initialization." % id)
                    # TODO, add behavior
        else:
            #standard tracking
            new_tracked_objects = []
            for obj in data.objects:
                pose = PoseStamped()
                pose.header = obj.header
                pose.pose = obj.pose.pose.pose
                
#                print "Absolute pose: %s" % pose
                pose = self._tf_listener.transformPose(self._rotating_tf_frame, pose)
#                print "Relative pose: %s" % pose
                radius = math.sqrt(pose.pose.position.x**2 + pose.pose.position.y**2)
                phase = math.atan2(pose.pose.position.y, pose.pose.position.x)
                print "Radius: %s Phase: %s" % (radius, phase)
                min_idx = -1
                min_dist = 0.1 #sys.float_info.max
                for i in range(len(self._tracked_objects)):
                    if self._tracked_objects[i].id == obj.id.id:
                        dist = self.polar_dist(radius, phase, self._tracked_objects[i].radius, self._tracked_objects[i].phase)
                        print "Dist == %s" % dist
                        if dist < min_dist:
                            min_dist = dist
                            min_idx = i
                            
                if min_idx != -1:
                    # add the current pose
                    self._tracked_objects[min_idx].poses.append(obj.pose)
                    self._tracked_objects[min_idx].stamps.append(obj.header.stamp)
                    # remove the object from the tracking set
                    # put it into the new set
                    new_tracked_objects.append(self._tracked_objects.pop(min_idx))
                else:
                    print "Creating new object."
                    # create a new object to track
                    # add it to the new set
                    tracked_object = TrackedObject()
                    tracked_object.id = obj.id.id
                    tracked_object.phase = phase
                    tracked_object.radius = radius
                    tracked_object.poses = [obj.pose]
                    tracked_object.stamps = [obj.header.stamp]                    
                    new_tracked_objects.append(tracked_object)                    
                
            
            #remove old objects > 10s
            time = rospy.Time.now().to_sec()
            self._tracked_objects = [x for x in self._tracked_objects if (time - x.poses[-1].header.stamp.to_sec() < 10.0)]                   
            # add back the new list to the old list
            self._tracked_objects.extend(new_tracked_objects)
            # smart stuff for old objects? stale count etc...
            # update motion model...  
            self.update_model()
            self.publish_markers()
            
            pass
                
        pass
    
    def polar_dist(self, r1, phi1, r2, phi2):
        return math.sqrt(r1**2 + r2**2 - 2*r1*r2*math.cos(phi1 - phi2))
    
    def tf_rate_callback(self, event):
        if not self._initialized:
            return
        
        with self._model_lock:
            self._previous_angle = self._rotation_speed * (event.current_real.to_sec() - self._last_tf_broadcast) + self._previous_angle
            self._tf_publisher.sendTransform(self._rotation_center, tf.transformations.quaternion_from_matrix(self._reference_frame), 
                                             event.current_real, self._intermediate_tf_frame, self._base_tf_frame)
            self._tf_publisher.sendTransform([0, 0, 0], tf.transformations.quaternion_about_axis(self._previous_angle, [0, 0, 1]),
                                             event.current_real, self._rotating_tf_frame, self._intermediate_tf_frame)
            self._last_tf_broadcast = event.current_real.to_sec()

    
    def start(self):
        rospy.init_node("rotating_object_tracker")
        
        # estimation service
        rospy.wait_for_service("estimate_rotation")
        self._estimate_rotation_service = rospy.ServiceProxy("estimate_rotation", EstimateRotation, True)
        self._marker_publisher = rospy.Publisher("rotating_objects", MarkerArray)
        
        # setup the recognized object callback
        rospy.Subscriber(self._recognized_objects_topic, RecognizedObjectArray, self.recognized_object_callback, queue_size=1)
        
        # setup the tf publisher
        self._tf_publisher = tf.TransformBroadcaster()
        rospy.Timer(rospy.Duration(0.1), self.tf_rate_callback)
        
        self._tf_listener = tf.TransformListener()
        rospy.loginfo("started")
        
        rospy.spin()
    
if __name__ == "__main__":
    tracker = Tracker()
    tracker.start()
    