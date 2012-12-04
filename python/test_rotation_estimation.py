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
import rospy
import numpy as np
import tf
from object_recognition_msgs.msg import RecognizedObjectArray
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Point, Vector3, PoseArray, Pose, PointStamped
from visualization_msgs.msg import MarkerArray, Marker
from object_tracker.srv import EstimateRotation, EstimateRotationResponse, EstimateRotationRequest
from copy import copy

class Tester:
    previous_poses = dict()
    estimate_rotation_service = EstimateRotation
    marker_publisher = rospy.Publisher
    pose_publisher = rospy.Publisher
    tf_publisher = tf.TransformBroadcaster
    tf_listener = tf.TransformListener
    rot_model = None
    previous_tf_callback = None
    
    
    def publish_markers(self, responses, header):
        markers = MarkerArray()
        
        id = 0
        for response in responses:
            # first, the center
            center_marker = Marker()
            center_marker.header = header
            center_marker.id = id
            id += 1
            center_marker.ns = "test_rotation_estimation"
            center_marker.action = Marker.ADD
            center_marker.lifetime = rospy.Duration(10)
            center_marker.type = Marker.ARROW
            arrow_end = Point()
            arrow_end.x = response.center.x + response.axis.x
            arrow_end.y = response.center.y + response.axis.y
            arrow_end.z = response.center.z + response.axis.z
    #        center_marker.pose.position.x = response.center.x
    #        center_marker.pose.position.y = response.center.y
    #        center_marker.pose.position.z = response.center.z
    #        center_marker.pose.orientation.x = response.axis.x
    #        center_marker.pose.orientation.y = response.axis.y
    #        center_marker.pose.orientation.z = response.axis.z
    #        center_marker.pose.orientation.w = 0
            center_marker.points = [response.center, arrow_end]
            center_marker.scale.x = 0.05
            center_marker.scale.y = 0.1
    #        center_marker.scale.z = 0.2
            center_marker.color.r = 1.0
            center_marker.color.g = 0
            center_marker.color.b = 0
            center_marker.color.a = 1.0
            
            markers.markers.append(center_marker)
        
        self.marker_publisher.publish(markers)
    
    def recognized_callback(self, data):
        #rospy.loginfo(rospy.get_name() + ': received callback with %d objects' % len(data.objects))
        if not data.objects:
            return
        
        header = data.objects[0].header
        for obj in data.objects:
            pose = PoseStamped()
            pose.header = obj.header
            pose.pose = obj.pose.pose.pose
            if obj.id.id in self.previous_poses:
                self.previous_poses[obj.id.id].append(pose)
            else:
                self.previous_poses[obj.id.id] = [pose]
            
        responses = []
        for id, poses in self.previous_poses.iteritems():
            if len(poses) > 15:
                print "Object %s: I have %d poses now. calling service." % (id, len(poses))
                request = EstimateRotationRequest()
                request.poses = poses
                try:
                    response = EstimateRotationResponse
                    response = self.estimate_rotation_service(request)
                    print "received a response: ", response
                    responses.append(response)
                except rospy.ServiceException, e:
                    print "Error! %s" % e
                 
            if len(poses) > 100:
                del poses[:-100]                
            
        if responses:
            response = EstimateRotationResponse()    
            for resp in responses:
                response.axis.x += resp.axis.x
                response.axis.y += resp.axis.y
                response.axis.z += resp.axis.z
                response.center.x += resp.center.x
                response.center.y += resp.center.y
                response.center.z += resp.center.z
                response.radius += resp.radius
                response.theta += resp.theta
            
            response.axis.x /= len(responses)
            response.axis.y /= len(responses)
            response.axis.z /= len(responses)
            response.center.x /= len(responses)
            response.center.y /= len(responses)
            response.center.z /= len(responses)
            response.radius /= len(responses)
            response.theta /= len(responses)
            
            self.rot_model = response
            if self.previous_tf_callback is None:
                self.previous_tf_callback = header.stamp
            
            responses = [response]
                        
            self.publish_markers(responses, header) # !!!
            
        markers = MarkerArray()
        id = 0        
        
        for obj in data.objects:
            center_marker = Marker()
            center_marker.header = copy(header)
            center_marker.header.frame_id = "/rotating_frame"
            center_marker.id = id
            id += 1
            center_marker.ns = "rotating_objects"
            center_marker.action = Marker.ADD
#                    center_marker.lifetime = rospy.Duration(10)

            try:
                point_transf = self.tf_listener.transformPoint("/rotating_frame", PointStamped(header=header, point=Point(obj.pose.pose.pose.position.x, obj.pose.pose.pose.position.y, obj.pose.pose.pose.position.z)))
                print point_transf
            except:
                return
            center_marker.type = Marker.SPHERE
            center_marker.pose.position.x = point_transf.point.x
            center_marker.pose.position.y = point_transf.point.y
            center_marker.pose.position.z = point_transf.point.z
            center_marker.pose.orientation.x = 0
            center_marker.pose.orientation.y = 0
            center_marker.pose.orientation.z = 0
            center_marker.pose.orientation.w = 1
            center_marker.scale.x = 0.05
            center_marker.scale.y = 0.05
            center_marker.scale.z = 0.05
            center_marker.color.r = 1.0
            center_marker.color.g = 0
            center_marker.color.b = 0
            center_marker.color.a = 1.0
            center_marker.frame_locked = True
            
            markers.markers.append(center_marker)

        
        self.marker_publisher.publish(markers)
            
    def timer_callback(self, event):
        if self.rot_model is None:
            return
        
        angle = (event.current_real.to_sec() - self.previous_tf_callback.to_sec()) * self.rot_model.theta
        
        axis = np.array([self.rot_model.axis.x, self.rot_model.axis.y, self.rot_model.axis.z])
        z_axis = axis
        if z_axis is not [0,1,0]:
            x_axis = np.cross(z_axis, [0,1,0])
        else:
            x_axis = np.cross(z_axis, [1,0,0])
            
        y_axis = np.cross(z_axis, x_axis)
        rot_matr =  np.array([[x_axis[0], x_axis[1], x_axis[2], 0], [y_axis[0], y_axis[1], y_axis[2], 0], [z_axis[0], z_axis[1], z_axis[2], 0], [0,0,0,1]])
        rot_matr = rot_matr.T

        self.tf_publisher.sendTransform([self.rot_model.center.x, self.rot_model.center.y, self.rot_model.center.z], tf.transformations.quaternion_from_matrix(rot_matr), 
                                        event.current_real, "/center_of_rotation", "/camera_rgb_optical_frame")
        self.tf_publisher.sendTransform([0, 0, 0], tf.transformations.quaternion_about_axis(angle, [0, 0, 1]), 
                                        event.current_real, "/rotating_frame", "/center_of_rotation")
        
        self.previous_tf_callback = event.current_real
        
        
        
        
    def start(self):
        rospy.init_node("test_rotation_estimation")
        rospy.Subscriber("/recognized_object_array", RecognizedObjectArray, self.recognized_callback, queue_size=1)
        self.marker_publisher = rospy.Publisher("rotation_markers", MarkerArray)
        self.pose_publisher = rospy.Publisher("rotating_poses", PoseArray)        
        
        # Setup server
        rospy.wait_for_service("estimate_rotation")
        self.estimate_rotation_service = rospy.ServiceProxy("estimate_rotation", EstimateRotation, True)
        rospy.Timer(rospy.Duration(0.1), self.timer_callback)
        
        self.tf_publisher = tf.TransformBroadcaster()
        self.tf_listener = tf.TransformListener()
        
        rospy.loginfo(rospy.get_name() + ": started.")
        
        rospy.spin()
        
    
if __name__ =='__main__':
    tester = Tester()
    tester.start()
