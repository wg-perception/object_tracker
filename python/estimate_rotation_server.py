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
# Original Authors: Steven Gray, Christian Dornhege, Georg Bartels, Jihoon Lee, John Schulmann
#                   Team 1, PR2 Workshop, Freiburg, Germany
# Edits: Tommaso Cavallari

"""
Adapted from http://www.scipy.org/Cookbook/Least_Squares_Circle
Input: points to fit to a circle, (x[], y[])
Output: circle center, 

"""

from numpy import *
import math
#import roslib; roslib.load_manifest('sushi_turntable')
import rospy
from object_tracker.srv import EstimateRotation, EstimateRotationResponse
from geometry_msgs.msg import PoseStamped, Pose, Point, Vector3, PoseWithCovarianceStamped, PoseWithCovariance
from scipy import optimize, linalg
import functools
from matplotlib import pyplot as p, cm, colors # only needed if want to plot separately

class CircleFinder:
    def __init__(self):
        self.center = None
        self.axis = None
        self.radius = None
        self.theta = None
        
    def calc_R(self, xc, yc, x, y):
        """ calculate the distance of each 3D point from the center (xc, yc) """
        return sqrt((x-xc)**2 + (y-yc)**2)

    def f_2(self, c, x_in, y_in):
        """ calculate the algebraic distance between the 3D points and the mean circle centered at c=(xc, yc) """
        x_c, y_c = c
        Ri = self.calc_R(x_c, y_c, x_in, y_in)
        return Ri - Ri.mean()

    # plotting functions
    def plot_all(self, xc_2, yc_2, R_2, x, y):
        p.close('all')
        """ 
        Draw data points, best fit circles and center for the three methods,
        and adds the iso contours corresponding to the fiel residu or residu2
        """
    
        f = p.figure( facecolor='white')  #figsize=(7, 5.4), dpi=72,
        p.axis('equal')
    
        theta_fit = linspace(-pi, pi, 180)
    
        x_fit2 = xc_2 + R_2*cos(theta_fit)
        y_fit2 = yc_2 + R_2*sin(theta_fit)
        p.plot(x_fit2, y_fit2, 'k--', label="leastsq", lw=2)
    
        # draw
        p.xlabel('x')
        p.ylabel('y')
    
        p.draw()
        xmin, xmax = p.xlim()
        ymin, ymax = p.ylim()
    
        vmin = min(xmin, ymin)
        vmax = max(xmax, ymax)
    
        # plot input data
        p.plot(x, y, 'ro', label='data', ms=8, mec='b', mew=1)
        p.legend(loc='best',labelspacing=0.1 )
    
        p.xlim(xmin=vmin, xmax=vmax)
        p.ylim(ymin=vmin, ymax=vmax)
    
        p.grid()
        p.title('Least Squares Circle')
       
        p.show()
    
    def fit_plane(self, x_in, y_in, z_in):
        # Plane with PCA
        points = array([x_in, y_in, z_in])
        center = points.mean(axis=1)
        points[0,:] -= center[0]
        points[1,:] -= center[1]
        points[2,:] -= center[2]
        covariance  = cov(points)
        
        eval, evec  = linalg.eig(covariance)
        ax_id = argmin(eval)
        plane_normal = evec[:, ax_id]
        plane_d = dot(center.T, plane_normal)
        
    #    print "center: %s" % center
    #    print "cov: %s" % covariance
    #    print "eval: %s" % eval
    #    print "evec: %s" % evec
    #    print "axe: %s" % ax_id
    #    print "normal: %s" % plane_normal
    #    print "plane_d: %s" % plane_d
        
        return array([plane_normal[0], plane_normal[1], plane_normal[2], plane_d])
    
    def project_points_to_plane(self, x_in, y_in, z_in, plane_coeffs):
        # define the origin (on the plane) as the point pointed by the coeff vector
        # d * (a, b, c)
        origin = (plane_coeffs[3]) * plane_coeffs[0:3]
    #    print "origin: %s" % origin
        
        # for each point project and obtain its x, y coords
        v_x = x_in - origin[0]
        v_y = y_in - origin[1]
        v_z = z_in - origin[2]
        
        dist = v_x * plane_coeffs[0] + v_y * plane_coeffs[1] + v_z * plane_coeffs[2]
    #    print "dist: %s" % dist
        
        proj_x = x_in - dist * plane_coeffs[0]
        proj_y = y_in - dist * plane_coeffs[1]
        proj_z = z_in - dist * plane_coeffs[2]
        
        return proj_x, proj_y, proj_z, origin
    
    def points3d_to_2d(self, x_in, y_in, z_in, plane_coeffs):
        # define arbitrary axis
        # the origin is defined by the plane_coeffs
        x_axis = None
        y_axis = None
        
        if array_equal(plane_coeffs[0:3], array([1,0,0])):
            x_axis = cross(plane_coeffs[0:3], array([0,1,0]))
        else:
            x_axis = cross(plane_coeffs[0:3], array([1,0,0]))
            
        y_axis = cross(plane_coeffs[0:3], x_axis)
        
        x_proj = []
        y_proj = []
        
        for i in range(len(x_in)):
            point = array([ x_in[i], y_in[i], z_in[i] ])
            x_proj.append(dot(point, x_axis))
            y_proj.append(dot(point, y_axis))
            
        return x_proj, y_proj, x_axis, y_axis
    
    def find_circle(self, x_in, y_in, plot_circle = False):
        # coordinates of the barycenter
        x_m = mean(x_in)
        y_m = mean(y_in)
    
        center_estimate = x_m, y_m
        center_2, ier = optimize.leastsq(self.f_2, center_estimate, args=(x_in, y_in))
    
        xc_2, yc_2 = center_2
        Ri_2       = self.calc_R(xc_2, yc_2, x_in, y_in)
        R_2        = Ri_2.mean()
        #residu_2   = sum((Ri_2 - R_2)**2)
        #residu2_2  = sum((Ri_2**2-R_2**2)**2)
        #ncalls_2   = f_2.ncalls
    
        # Summary
        #fmt = '%-22s %10.5f %10.5f %10.5f %10d %10.6f %10.6f %10.2f'
        #print ('\n%-22s' +' %10s'*7) % tuple('METHOD Xc Yc Rc nb_calls std(Ri) residu residu2'.split())
        #print '-'*(22 +7*(10+1))
    
        #print  fmt % (method_2 , xc_2 , yc_2 , R_2 , ncalls_2 , Ri_2.std() , residu_2 , residu2_2 )
        
        if plot_circle:
            self.plot_all(xc_2, yc_2, R_2, x_in, y_in)
    
        return xc_2, yc_2, R_2
    
    def find_circle(self, x_in,y_in, times, plot_circle=False):
        # coordinates of the barycenter
        x_m = mean(x_in)
        y_m = mean(y_in)
    
        center_estimate = x_m, y_m
        center_2, ier = optimize.leastsq(self.f_2, center_estimate, args=(x_in, y_in))
    
        xc_2, yc_2 = center_2
        Ri_2       = self.calc_R(xc_2, yc_2, x_in, y_in)
        R_2        = Ri_2.mean()
        #residu_2   = sum((Ri_2 - R_2)**2)
        #residu2_2  = sum((Ri_2**2-R_2**2)**2)
        #ncalls_2   = f_2.ncalls
    
        # Summary
        #fmt = '%-22s %10.5f %10.5f %10.5f %10d %10.6f %10.6f %10.2f'
        #print ('\n%-22s' +' %10s'*7) % tuple('METHOD Xc Yc Rc nb_calls std(Ri) residu residu2'.split())
        #print '-'*(22 +7*(10+1))
    
        #print  fmt % (method_2 , xc_2 , yc_2 , R_2 , ncalls_2 , Ri_2.std() , residu_2 , residu2_2 )
        
        ang_vel = 0
    
        x_rel = x_in - xc_2
        y_rel = y_in - yc_2
          
        angle_history = time_history = array([])
        for i in range(len(x_rel)):
            if not i == 0:
                if times[i] == times[i-1]:
                    rospy.loginfo('Skipping frame since it has same timestamp as last frame')
                    continue
            angle_history = append(angle_history, math.atan2(y_rel[i], x_rel[i]))
            time_history = append(time_history, times[i])
    
        angle_diff = angle_history[1:] - angle_history[:-1]
        time_diff = time_history[1:] - time_history[:-1]
    
        for i in range(len(angle_diff)):
            if abs(angle_diff[i]) > math.pi:    
                angle_diff[i] = -1 * sign(angle_diff[i])*2*math.pi + angle_diff[i]
    
        vel = angle_diff / time_diff
        ang_vel = mean(vel)
    
        if plot_circle:
            self.plot_all(xc_2, yc_2, R_2, x_in, y_in)
    
        return xc_2, yc_2, R_2, ang_vel

    def find_circle_posestamped(self, req):
        pose_stamped_list = req.poses
        
        if len(pose_stamped_list) < 5:
            print 'Not enough poses to estimate a rotation'
            return EstimateRotationResponse(success=False)
        
        x_in = [] #array('f')
        y_in = [] #array('f')
        z_in = [] #array('f')
        times_in = []

        for i in range(len(pose_stamped_list)):
            times_in.append(pose_stamped_list[i].header.stamp.to_sec())
            x_in.append(pose_stamped_list[i].pose.pose.position.x)
            y_in.append(pose_stamped_list[i].pose.pose.position.y)
            z_in.append(pose_stamped_list[i].pose.pose.position.z)
            
        # 1st thing: find the supporting plane
        plane_coeffs = self.fit_plane(x_in, y_in, z_in)
        #print "plane coeffs: %s" % plane_coeffs
        
        # 2nd thing: project the points on the plane and find their 2d coords wrt the "origin" point on the plane in an arbitrary reference frame
        proj_x, proj_y, proj_z, origin = self.project_points_to_plane(x_in, y_in, z_in, plane_coeffs)
        x_proj2d, y_proj2d, x_axis, y_axis = self.points3d_to_2d(proj_x, proj_y, proj_z, plane_coeffs)
        
        # 3rd: now find the circle.
        c_x, c_y, self.radius, self.theta = self.find_circle(x_proj2d, y_proj2d, times_in, False)
        
        # c_x and c_y are relative to the origin on the plane, convert them back to world coords
        c_vector = origin + x_axis * c_x + y_axis * c_y

        self.center = Point()
        self.center.x = c_vector[0]
        self.center.y = c_vector[1]
        self.center.z = c_vector[2]
        
        axis = array([plane_coeffs[0], plane_coeffs[1], plane_coeffs[2]])
        #c_vector points towards the rotation center
        if dot(c_vector, axis) > 0:
            axis = -axis
            self.theta = -self.theta
        
        self.axis = Vector3()
        self.axis.x = axis[0]
        self.axis.y = axis[1]
        self.axis.z = axis[2]        
        
        response = EstimateRotationResponse()
        response.success = True
        response.center = self.center
        response.axis = self.axis
        response.radius = self.radius
        response.theta = self.theta        
        
        return response
    
    def start(self):
        rospy.init_node('estimate_rotation_server')
        s = rospy.Service('estimate_rotation', EstimateRotation, self.find_circle_posestamped)
        print "Ready to estimate circles."
        rospy.spin()
    

if __name__ == "__main__":
    server = CircleFinder()
    server.start()
    