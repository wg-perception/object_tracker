#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Willow Garage, Inc.
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
from geometry_msgs.msg import PoseStamped
from object_recognition_msgs.msg import RecognizedObjectArray, RecognizedObject, ObjectId
from object_tracker.msg import RotationParameters, RotatingObjects
from object_tracker import BaseTracker, TrackedObject, MotionParameters
from object_tracker.estimate_rotation_server import CircleFinder

class RotationParameters(MotionParameters):
    radius = 0.0
    phase = 0.0
    
class RotationTracker(BaseTracker):
    _tf_publisher = tf.TransformBroadcaster
    _tf_listener = tf.TransformListener
    _rotation_publisher = rospy.Publisher
    _rotating_objects_publisher = rospy.Publisher
    _tracker_name = ""
    
    _circle_finder = CircleFinder
    _model_lock = threading.Lock
    
    _base_tf_frame = ""
    _intermediate_tf_frame = "/rotation_center"
    _rotating_tf_frame = "/rotating_objects"
    
    _min_poses_to_consider_an_object = 0
    _min_poses_for_estimation = 0
    _max_poses_for_object = 0

    _reference_frame = np.array
    _rotation_center = np.array
    _rotation_axis = np.array
    _rotation_speed = np.array
    _previous_angle = 0.0
    _last_tf_broadcast = 0.0 
    
    _axis_covariance = np.array
    _axis_time_covariance = np.array
    _center_covariance = np.array
    _center_time_covariance = np.array
    _speed_std_dev = 0.0
    _speed_time_std_dev = 0.0  
    
    def __init__(self, name, **kwargs):
        self._tracker_name = name
        self._circle_finder = CircleFinder()
        self._model_lock = threading.Lock()
        
        self._rotation_center = np.zeros(3)
        self._rotation_axis = np.zeros(3)
        self._rotation_speed = np.zeros(1)
        self._previous_angle = 0.0
        self._last_tf_broadcast = 0.0   
        self._reference_frame = np.identity(4)
        self._min_poses_to_consider_an_object = 5
        self._min_poses_for_estimation = 10
        self._max_poses_for_object = 50
        
        self._axis_covariance = np.identity(3).flatten().tolist()
        self._axis_time_covariance = np.identity(3).flatten().tolist()
        self._center_covariance = np.identity(3).flatten().tolist()
        self._center_time_covariance = np.identity(3).flatten().tolist()
        self._speed_std_dev = 0.0
        self._speed_time_std_dev = 0.0
        
        if 'rotation_center_frame' in kwargs:
            self._intermediate_tf_frame = kwargs['rotation_center_frame']
            print('Set intermediate_tf_frame to %s' % self._intermediate_tf_frame)
        if 'rotating_frame' in kwargs:
            self._rotating_tf_frame = kwargs['rotating_frame']
            print('Set rotating_frame to %s' % self._rotating_tf_frame) 
            
    
    def broadcast_tf(self, time, objects):
        """ Publish TF data: a static frame for the center and axis of rotation, a moving rotating frame and an unique frame for each tracked object. """
        # the lock on the model should have been acquired outside
        self._previous_angle = self._rotation_speed[-1] * (time.to_sec() - self._last_tf_broadcast) + self._previous_angle
        self._tf_publisher.sendTransform(self._rotation_center[-1,:], tf.transformations.quaternion_from_matrix(self._reference_frame), 
                                         time, self._intermediate_tf_frame, self._base_tf_frame)
        self._tf_publisher.sendTransform([0.0, 0.0, 0.0], tf.transformations.quaternion_about_axis(self._previous_angle, [0.0, 0.0, 1.0]),
                                         time, self._rotating_tf_frame, self._intermediate_tf_frame)
        
        for obj in objects:
            if len(obj.poses) < self._min_poses_to_consider_an_object:
                continue
            
            obj_x = obj.motion_parameters.radius * math.cos(obj.motion_parameters.phase)
            obj_y = obj.motion_parameters.radius * math.sin(obj.motion_parameters.phase)
            self._tf_publisher.sendTransform([obj_x, obj_y, 0.0], [0.0, 0.0, 0.0, 1.0], time, obj.recognized_object.header.frame_id, self._rotating_tf_frame)
        
        self._last_tf_broadcast = time.to_sec()
        
    def publish_messages(self, objects):
        if self._rotation_publisher.get_num_connections() > 0:
            self.publish_rotation_msg(objects)
        if self._rotating_objects_publisher.get_num_connections() > 0:
            self.publish_rotating_objects(objects)
        
    def update_model(self, header, objects):
        """
        Update the rotation parameters using the newly acquired poses for the tracked objects.

        For each tracked object the rotation model is estimated independently; the results are then combined in order to
        obtain a more robust set of rotation parameters.
        """
        num_models = 0
        new_axii = []
        new_centers = []
        new_speeds = []       
        
        for obj in objects:
            if len(obj.poses) > self._min_poses_for_estimation:
                success, center, axis, radius, speed = self._circle_finder.find_circle_pose_with_covariance_stamped(obj.poses)
                if success:
                    new_axii.append(axis)
                    new_centers.append(center)
                    new_speeds.append(speed)
                    num_models += 1
#                        print "Response: ", response
                        #tenerlo qua
                        
            if len(obj.poses) > self._max_poses_for_object:
                del obj.poses[:-self._max_poses_for_object]
                  
        if num_models > 0:                    
            new_axis = np.mean(new_axii, axis=0)
            new_center = np.mean(new_centers, axis=0)
            new_speed = np.mean(new_speeds)                           
            
            # fix up vector (and speed)
            if np.dot(new_axis, [ 0.0, 0.0, 1.0 ]) < 0:
                new_axis = -new_axis
                new_speed = -new_speed
            if new_axis is not [ 0.0, 1.0, 0.0 ] and new_axis is not [ 0, -1, 0 ]:
                x_axis = np.cross(new_axis, [ 0.0, 1.0, 0.0 ])
            else:
                x_axis = np.cross(new_axis, [ 1.0, 0.0, 0.0 ])
                
            y_axis = np.cross(new_axis, x_axis)
            rot_matr =  np.array([ [ x_axis[0], y_axis[0], new_axis[0], 0.0 ], 
                                   [ x_axis[1], y_axis[1], new_axis[1], 0.0 ],
                                   [ x_axis[2], y_axis[2], new_axis[2], 0.0 ],
                                   [ 0.0,       0.0,       0.0,         1.0 ] ])
            
            with self._model_lock:
                self._rotation_center = np.vstack((self._rotation_center, new_center))
                self._rotation_axis = np.vstack((self._rotation_axis, new_axis))
                self._rotation_speed = np.vstack((self._rotation_speed, new_speed))
                
                # remove in excess data
                if self._rotation_center.shape[0] > self._max_poses_for_object:
                    self._rotation_center = self._rotation_center[-self._max_poses_for_object:-1,:]
                                   
                if self._rotation_axis.shape[0] > self._max_poses_for_object:
                    self._rotation_axis = self._rotation_axis[-self._max_poses_for_object:-1,:]
                                    
                if self._rotation_speed.shape[0] > self._max_poses_for_object:
                    self._rotation_speed = self._rotation_speed[-self._max_poses_for_object:-1]               
                    
                self._reference_frame = rot_matr
                
                # update stats
                if len(new_centers) > 1:
                    self._center_covariance = np.cov(new_centers, rowvar=0).flatten().tolist()
                else:
                    self._center_covariance = np.identity(3).flatten().tolist()
                    
                if self._rotation_center.shape[0] > 1:
                    self._center_time_covariance = np.cov(self._rotation_center, rowvar=0).flatten().tolist()
                else:
                    self._center_time_covariance = np.identity(3).flatten().tolist()  
                    
                if len(new_axii) > 1:
                    self._axis_covariance = np.cov(new_axii, rowvar=0).flatten().tolist()
                else:
                    self._axis_covariance = np.identity(3).flatten().tolist()
                    
                if self._rotation_axis.shape[0] > 1:
                    self._axis_time_covariance = np.cov(self._rotation_axis, rowvar=0).flatten().tolist()
                else:
                    self._axis_time_covariance = np.identity(3).flatten().tolist() 
                    
                self._speed_std_dev = np.std(self._rotation_speed)
                self._speed_time_std_dev = np.std(self._rotation_speed)
                
                # now or at the time the detection was performed?
                # broadcast the updated transformations
                self.broadcast_tf(rospy.Time.now(), objects)
#                self.broadcast_tf(header.stamp)
                
            rospy.logdebug("Updated model: center: %s axis: %s speed: %s" % (new_center, new_axis, new_speed))
            
            # Update already tracked objs to reflect the new model
            rospy.logdebug("There are %s tracked objects." % len(objects))
            for obj in objects:
                obj_pose = PoseStamped()
                obj_pose.header = obj.poses[-1].header
                obj_pose.pose = obj.poses[-1].pose.pose
                try: 
                    obj_pose = self._tf_listener.transformPose(self._rotating_tf_frame, obj_pose)
                    obj.motion_parameters.radius = math.sqrt(obj_pose.pose.position.x**2 + obj_pose.pose.position.y**2)
                    obj.motion_parameters.phase = math.atan2(obj_pose.pose.position.y, obj_pose.pose.position.x)
                except tf.Exception, e:
                    rospy.logwarn("%s" % e)
                    
        
    def init_model_from_object(self, object):
        """
        Initializes the rotation model with a rough estimate based only on one object.

        Once a single object has been identified in multiple subsequent recognition results, a circle is fit through its poses and some starting rotation parameters
        are determined in order to track more objects in the future and improve the accuracy.

        Args:
            object: a TrackedObject

        Returns:
            a boolean value indicating whether the model estimation was successful or not
        """
        if len(object.poses) <= self._min_poses_for_estimation:
            return False
        
        success, rotation_center, z_axis, radius, speed = self._circle_finder.find_circle_pose_with_covariance_stamped(object.poses)
        
        if not success:
            return False

        if np.dot(z_axis, [0.0, 0.0, 1.0]) < 0:
            z_axis = -z_axis
            speed = -speed
        if z_axis is not [0.0,1.0,0.0] and z_axis is not [0.0, -1.0, 0.0]:
            x_axis = np.cross(z_axis, [0.0,1.0,0.0])
        else:
            x_axis = np.cross(z_axis, [1.0,0.0,0.0])
            
        y_axis = np.cross(z_axis, x_axis)
        rot_matr =  np.array([[x_axis[0], y_axis[0], z_axis[0], 0.0], 
                              [x_axis[1], y_axis[1], z_axis[1], 0.0],
                              [x_axis[2], y_axis[2], z_axis[2], 0.0],
                              [0.0,0.0,0.0,1.0]])
        
        
        # initialize the object
        object.motion_parameters.radius = radius
        obj_pose = self.pose_to_array(object.poses[-1]) - rotation_center
        obj_x = np.dot(x_axis, obj_pose)
        obj_y = np.dot(y_axis, obj_pose)
        object.motion_parameters.phase = math.atan2(obj_y, obj_x)

        with self._model_lock:
            self._rotation_center = np.array([rotation_center])
            self._rotation_axis = np.array([z_axis])
            self._rotation_speed = np.array([speed])
            self._reference_frame = rot_matr
#            self._initialized = True 
#            self._model_valid = True
            self._last_tf_broadcast = object.poses[-1].header.stamp.to_sec()   
        
        rospy.loginfo("Initialization successful.")
            
        return True
    
        
    def find_closest_object(self, object_parameters, object_list, max_dist = sys.float_info.max):
        """
        Find the closest object from a list using polar distance.

        Args:
            object_parameters: the MotionParameters to which the nearest neighbor has to be found
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
            distance = self.compute_distance(object_parameters, obj.motion_parameters)
            if distance < min_dist:
                closest_obj = obj
                min_dist = distance
        
        return closest_obj  
          
    def set_parameters(self, config):  
         # tracking params
        self._min_poses_to_consider_an_object = config['min_poses_for_tracking']
        self._max_poses_for_object = config['max_poses_for_object']
        self._base_tf_frame = config['fixed_frame']
         
#        # frames
#        if not (self._base_tf_frame == config['fixed_frame'] 
#                and self._intermediate_tf_frame == config['rotation_center_frame'] 
#                and self._rotating_tf_frame == config['rotating_frame']):
#            self._base_tf_frame = config['fixed_frame']
#            self._intermediate_tf_frame = config['rotation_center_frame']
#            self._rotating_tf_frame = config['rotating_frame']
#            
#            return True
#        
#        return False
        
    def start(self, tf_publisher, tf_listener):   
        
        # Publishers
        self._rotation_publisher = rospy.Publisher("rotating_objects", RotatingObjects)
        self._rotating_objects_publisher = rospy.Publisher("recognized_rotating_objects", RecognizedObjectArray)
        
        # TF
        self._tf_publisher = tf_publisher        
        self._tf_listener = tf_listener
        
        rospy.loginfo("Rotation Tracker started...")
        
    def publish_rotation_msg(self, objects):  
        """
        Publish a message containing the rotation parameters, the estimation confidences and the list of tracked objects.

        Args:
            objects: a list of tracked objects
        """
        rotating_objs = RotatingObjects() 
              
        rotation_params = RotationParameters()
        rotation_params.header.frame_id = self._base_tf_frame
        rotation_params.header.stamp = rospy.Time.now()
        
        rotation_params.center.x = self._rotation_center[-1, 0]
        rotation_params.center.y = self._rotation_center[-1, 1]
        rotation_params.center.z = self._rotation_center[-1, 2]
        
        rotation_params.center_covariance = self._center_covariance
        rotation_params.center_time_covariance = self._center_time_covariance           
        
        rotation_params.axis.x = self._rotation_axis[-1, 0]
        rotation_params.axis.y = self._rotation_axis[-1, 1]
        rotation_params.axis.z = self._rotation_axis[-1, 2]
        
        rotation_params.axis_covariance = self._axis_covariance
        rotation_params.axis_time_covariance = self._axis_time_covariance      
        
        rotation_params.speed = self._rotation_speed[-1]
        rotation_params.speed_std_dev = self._speed_std_dev
        rotation_params.speed_time_std_dev = self._speed_time_std_dev
        
        rotating_objs.rotation_parameters = rotation_params
        
        for obj in objects:
            if len(obj.poses) < self._min_poses_to_consider_an_object:
                continue
            
            rotating_objs.objects.append(obj.recognized_object)
            rotating_objs.radius.append(obj.motion_parameters.radius)
            rotating_objs.phase.append(obj.motion_parameters.phase)
        
        self._rotation_publisher.publish(rotating_objs)
        
    def publish_rotating_objects(self, objects):
        """ 
        Publish an object_recognition_msgs/RecognizedObjectArray containing the tracked 
        objects with the poses expressed using the rotating reference frame. 
        """
        recognized_objects = RecognizedObjectArray()
        
        now = rospy.Time().now()
        recognized_objects.header.frame_id = self._base_tf_frame
        recognized_objects.header.stamp = now
        
        for obj in objects:
            if len(obj.poses) < self._min_poses_to_consider_an_object:
                continue
            # update the timestamp of the object for our subscribers
            obj.recognized_object.header.stamp = now
            
            recognized_objects.objects.append(obj.recognized_object)
        
        self._rotating_objects_publisher.publish(recognized_objects) 
        
    def pose_to_array(self, pose):
        """ Convert a PoseWithCovarianceStamped into a numpy.array. """
        return np.array([ pose.pose.pose.position.x, 
                          pose.pose.pose.position.y, 
                          pose.pose.pose.position.z ])
    def polar_dist(self, r1, phi1, r2, phi2):
        """ Compute the polar distance between two points. """
        return math.sqrt(r1**2 + r2**2 - 2*r1*r2*math.cos(phi1 - phi2))
    
    def compute_distance(self, params_a, params_b):
        return self.polar_dist(params_a.radius, params_a.phase, params_b.radius, params_b.phase)
    
    def get_motion_parameters(self, pose):
        # transform the pose in the rotating coordinates so we can extract the parameters
        try:
            pose = self._tf_listener.transformPose(self._rotating_tf_frame, pose)
        except tf.Exception, e:
            rospy.logwarn("Tf exception: %s" % e)
            return None
        
        parameters = MotionParameters()                
        parameters.radius = math.sqrt(pose.pose.position.x**2 + pose.pose.position.y**2)
        parameters.phase = math.atan2(pose.pose.position.y, pose.pose.position.x)
        rospy.logdebug("Radius: %s Phase: %s" % (parameters.radius, parameters.phase))
        
        return parameters
