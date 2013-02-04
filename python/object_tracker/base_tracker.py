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
import rospy

class BaseTracker:
    def broadcast_tf(self, time, objects):
        """ 
        Publish the Tracker's specific TF data. 
        
        Args:
            time: the timestamp to use when broadcasting the frames.
            objects: a list of TrackedObject for which the TF frames have to be broadcasted.
        """
        rospy.logwarn('broadcast_tf not implemented.')
        
    def publish_messages(self, objects):
        """ 
        Publish the Tracker's specific messages. 
        
        Args:
            objects: a list of TrackedObject.
        """
        rospy.logwarn('publish_messages not implemented.')
        
    def update_model(self, header, objects):
        """
        Update the motion parameters using the newly acquired poses for the tracked objects.
        
        Args:
            header: a Header used to update the model
            objects: a list of TrackedObject.
        """
        rospy.logwarn('update_model not implemented.')
        
    def init_model_from_object(self, object):
        """
        Initializes the model with a rough estimate based only on one object.

        Args:
            object: a TrackedObject

        Returns:
            a boolean value indicating whether the model estimation was successful or not
        """
        rospy.logwarn('init_model_from_object not implemented.')
        
    def find_closest_object(self, object_parameters, object_list, max_dist = sys.float_info.max):
        """
        Find the closest object from a list using the tracker's specific distance.

        Args:
            object_parameters: the MotionParameters to which the nearest neighbor has to be found
            object_list: a list of TrackedObject
            max_dist: the maximum distance allowed between the object and its closest neighbor

        Returns:
            the closest TrackedObject if found or None
        """
        rospy.logwarn('find_closest_object not implemented.')
        
    def get_motion_parameters(self, pose):
        """
        Transform an absolute pose in a pose relative to the moving reference frame and return the coordinates
        using this specific MotionEstimation parameters.
        
        Args:
            pose: a PoseStamped
            
        Return:
            an instance of Parameters that expresses the input pose.
        """
        rospy.logwarn('get_motion_parameters not implemented.')
        
    def compute_distance(self, params_a, params_b):
        """ 
        Compute the tracker's specific distance between two instances of MotionParameter. 
        
        Args: 
            params_a, params_b: two instances of MotionParameters
        
        Return:
            the distance between the two parameters.
        """
        rospy.logwarn('compute_distance not implemented.')
        
    def set_parameters(self, config):   
        """ 
        Function called by the ObjectTracker when its parameters are dynamically changed. 
        
        Args:
            config: a dictionary containing the new parameters.
        """
        rospy.logwarn('set_parameters not implemented.')
        
    def start(self, tf_publisher, tf_listener):  
        """
        Configure the motion estimator.
        
        Args:
            tf_publisher: TF publisher used to broadcast the motion specific frames.
            tf_listener: TF listener used to query past reference frames.
        """ 
        rospy.logwarn('start not implemented.')
