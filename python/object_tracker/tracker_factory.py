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

import argparse
import sys
import importlib
import yaml
import traceback
from object_tracker import BaseTracker

def create_tracker():
    parser = argparse.ArgumentParser(description='Moving objects tracker with configurable motion estimation algorithm.')
    parser.add_argument('-c', dest='config_file', help='The path to the motion estimation configuration file.', required=True, 
                        type=argparse.FileType('r'))
    args = parser.parse_known_args()[0]
    
    try:
        tracker_params = yaml.safe_load(args.config_file)
    except:
        print 'Unable to parse the configuration file.'
        sys.exit(1)
        
    if len(tracker_params) != 1:
        print 'The configuration file must contain only one estimator definition.'
        sys.exit(1)
    
    try:   
        for tracker_name, parameters in tracker_params.items():
            if 'module' not in parameters:
                raise Exception('You need a "module" parameter to define where your estimator "%s" is.' % tracker_name)
            if 'type' not in parameters:
                raise Exception('You need a "type" parameter to define what your estimator "%s" is.' % tracker_name)
            cell_module = importlib.import_module(parameters['module'])
            cell_class = getattr(cell_module, parameters['type'])
    
            if not issubclass(cell_class, BaseTracker):
                print 'The class "%s" is not "%s"\'s subclass.' % (parameters['type'], BaseTracker.__name__)
                sys.exit(1)
                
            # instantiate the tracker
            try:
                if 'parameters' in parameters:
                    tracker = cell_class(tracker_name, **parameters['parameters'])
                else:
                    tracker = cell_class(tracker_name)
            except TypeError as err:
                exc_type, exc_value, exc_traceback = sys.exc_info()
                err = traceback.format_exception(exc_type, exc_value, exc_traceback)
                raise Exception('Could not initialize estimator "%s" because of: %s' % (tracker_name, ''.join(err)))
    except:
        exc_type, exc_value, exc_traceback = sys.exc_info()
        print exc_value
        sys.exit(1)
        
    return tracker