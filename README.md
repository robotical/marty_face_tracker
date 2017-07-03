# marty_face_tracker

Face, eye and smile tracking over ROS using OpenCV.

***Note:*** a camera is required.

Installation
===

To install, simply clone this repo into your workspace's ``.../src`` folder and compile using ``catkin_make``.

Once done, run the launch file: ```roslaunch marty_face_tracker face_tracking.launch```.

Configuration
===

The file ```face_tracking.launch``` contains several parameters which can be modified:

	video_output
    face_classifier
    eye_classifier
    smile_classifier

The first, ```video_output```, is a boolean value defining whether Marty publishes visualisation of the tracking being performed, such as a rectangle being drawn on a detected face. Setting this to ```true``` will tell Marty to advertise these topics and vice-versa.

The next three define the classifier to use when attempting to detect a certain feature. Bundled in the package within ```.../classifiers``` are a selection of some pre-made open source classifiers. Creating your own is possible, though outwith the scope of this package. To change the classifier, simply change the filename defined within these parameters.

This ```launch``` file also loads a separate parameter file, ```detection_params.yaml```, which defines the parameters used within the package's feature detection methods. Take a look at the comments within this file for further clarification.

Topics
===

### Publishers:

	Video:

    /marty/face_tracking/faces
    /marty/face_tracking/smiles
    /marty/face_tracking/eyes

    Co-ordinates:

    /marty/face_tracking/faces_centroid
    /marty/face_tracking/smiles_centroid
    /marty/face_tracking/eyes_centroid

The format of the co-ordinate message are three x-y-z arrays of floats, defining the centroid co-ordinate of detected features.

	float32[] x
	float32[] y
    float32[] z

For example, the co-ordinates of the first face would be at ```x[0]```, ```y[0]```, ```z[0]``` and so on.

This is defined within ```CentroidMsg.msg```, within the [marty_msgs](https://github.com/robotical/marty_msgs) package.

### Subscribers:

	/marty/camera/image  
