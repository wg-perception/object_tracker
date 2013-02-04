object_tracker
==============
This package allows for the tracking of moving objects
and the estimation of the relevant motion parameters.
The package gives the infrastructure to estimate the motion model and track
the objects using different motion model estimators (e.g: a rotating motion, 
a linear motion...).
A rotation model estimator is provided: the estimated motion parameters 
are the center of rotation, the rotation axis and the rotation speed.

How it works
------------
The `object_tracker` package uses the the output provided by the ROS
[Object Recognition Kitchen]. This package examines subsequent detection
results looking for moving objects and, if found, is able to determine
the relevant motion parameters; in the case of the rotation estimator,
the relevant parameters are the center and axis of rotation
and the angular speed. Some statistical indicators (covariances) are also
output to give an idea of the stability in time and spatial location of
the estimated parameters.

[Object Recognition Kitchen]: http://ecto.willowgarage.com/recognition/

### Inputs
There is only one input needed, in the form of a topic of type
`object_recognition_msgs/RecognizedObjectArray`
The topic name is by default `recognized_object_array`

### Outputs
This package has various outputs:

- TF data: the center of rotation determines a moving (rotating)
  reference frame, each tracked object in addition is associated to a
  static reference frame whose coordinates are expressed in relation to
  the rotating frame expressed by the center of rotation.
- `rotating_objects_markers`: topic of type 
  `visualization_msgs/MarkerArray`, contains a spherical marker for
  each of the objects currently being 
  tracked.
- `rotating_objects`: topic of type `object_tracker/RotatingObjects`; the
  message contains the current rotation parameters, the covariances and
  a list of the tracked objects including their respective radii and
  phases.
- `recognized_rotating_objects`: topic of type
  `object_recognition_msgs/RecognizedObjectArray`, contains the same data
  as the input `recognized_object_array` but with the object poses
  expressed using the rotating reference frame instead of the
  original TF frame.

Installation
------------
###Dependencies
This package depends on the [Object Recognition Kitchen] hence you need to have
in your ROS setup the following packages:

- `object_recognition_core`
- `object_recognition_msgs`
- `object_recognition_ros`

You also need an object recognition algorithm; while you can use any
algorithm of your choice the suggested one is 
`object_recognition_tabletop`.

### Setup
Inside your Catkin workspace clone this repository:

	$ git clone https://github.com/wg-perception/object_tracker.git
Then simply run 

	$ catkin_make
to generate the message, service and dynamic_reconfigure files.

Launch
------
To launch the tracker with the default parameters and having it launch for you the tabletop detector simply run:

	$ roslaunch object_tracker track.launch

Alternatively, if you prefer launching a different object detector or you want
to use a different motion estimation you can do this:

	$ rosrun object_recognition_ros server -c /path/to/recognition/config/file
	$ rosrun object_tracker multi_object_tracker.py -c /path/to/estimator/config/file

### Parameters
There are various parameters that can be set to fine tune the rotation 
model estimation; the default values should be good for a variety of 
practical cases but, if you want to change the behavior, you can do so 
by running:

	$ rosrun dynamic_reconfigure reconfigure_gui /objects_tracker

To understand the meaning of each parameter simply hover your mouse on
top of its name or have a look inside [cfg/ObjectTracker.cfg](cfg/ObjectTracker.cfg).

Different motion estimators
---------------------------
If you want you can create a different motion estimator to use with the `ObjectTracker`.
To do so simply create a new Python class subclassing [`BaseTracker`](python/object_tracker/base_tracker.py)
and fill the methods according to your specific motion model. 
You can look into the current 
[rotation estimator class](python/object_tracker/rotation_tracker.py)
to get an idea of the expected behavior.

To run your newly created motion estimator you need to pass to the `multi_object_tracker.py`
script the path to a yaml configuration file that specifies which estimation class 
you want to use.

The configuration file should have a structure similar to this:

```yaml
TrackerName: # can be anything
	type: PythonClassName 
	module: Python.Module.Of.Your.Estimator
	parameters:
		# eventual parameters using the 
		key: value # syntax
```

Then you can launch the tracker using the following command: 

	$ rosrun object_tracker multi_object_tracker.py -c /path/to/estimator/config/file
