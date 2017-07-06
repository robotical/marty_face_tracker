/**
 * @file      face_tracking.cpp
 * @brief     Marty tracks faces, eyes and smiles
 * @author    Helmi Fraser <helmi@robotical.io>
 * @date      2017-07-03
 * @copyright (Apache) 2016 Robotical Ltd.
 */

#include <marty_face_tracker/face_tracking.hpp>

/**
 * @brief       FaceTracker object constructor
 * @details      Loads ROS parameters, desired classifiers and  sets up ROS
 */

FaceTracker::FaceTracker(ros::NodeHandle &nh) : it_(nh) {
  this->loadParams();
  this->loadClassifiers();
  this->rosSetup();
}

FaceTracker::~FaceTracker() {}

/**
 * @brief       Load ROS parameters
 * @details     Loads runtime parameters from parameter server. Loads classifier names,
 *              whether video output is set, and face/eye/smile detection parameters.
 */

void FaceTracker::loadParams() {
  bool fail = false;
  bool camera_detected = false;

  nh_.param<bool>("camera", camera_detected, false);

  // Loads classifier names and will signal an error if unset
  nh_.param<std::string>("face_classifier", face_cascade, "ERROR");
  nh_.param<std::string>("eye_classifier", eye_cascade, "ERROR");
  nh_.param<std::string>("smile_classifier", smile_cascade, "ERROR");

  // If true, Marty will not publish video over ROS, just co-ordinate data
  nh_.param<bool>("video_output", video_output, false);

  if (video_output == true) {
    ROS_WARN("video_output set to true");
  } else {
    ROS_WARN("video_output set to false");
  }

  std::vector<std::string> param1 = {"face", "eye", "smile"};
  std::vector<std::string> param2 = {"_scale_factor", "_min_neighbours",
                                     "_min_size", "_max_size"};

  // Array of floats holding default values for face/eye/smile detection. In ascending order
  // of scale factor, minimum neighbors, minimum size and maximum size
  float default_vals[4] = {1.1, 3, 30, 30};
  bool default_param = true;
  // If true, will load the default parameters instead of the config file's
  nh_.param<bool>("ignore", default_param, true);

  // Iterates through the config file's parameters and saves them to detection_parameters,
  // else copies the default_vals
  if (default_param != true) {
    ROS_WARN("'ignore' param set to false, loading  detection parameters from "
             "config file\n");
    for (size_t i = 0; i < 3; i++) {
      for (size_t j = 0; j < 4; j++) {
        nh_.param<float>(param1[i] + param2[j], detection_parameters[i][j],
                         default_vals[j]);
      }
    }
  } else {
    ROS_WARN("'ignore' param set to true, loading default values\n");
    memcpy(detection_parameters[0], default_vals,
           sizeof(detection_parameters[0]));
    memcpy(detection_parameters[1], default_vals,
           sizeof(detection_parameters[1]));
    memcpy(detection_parameters[2], default_vals,
           sizeof(detection_parameters[2]));
  }

  if (camera_detected == false) {
    ROS_ERROR("Raspicam undetected.\n");
    ROS_WARN("Has the raspicam node been instantiated? If not, check ros_marty/launch/marty.launch\n");
    fail = true;
  }

  // Error handling for unset classifiers
  if (face_cascade.compare("ERROR") == 0) {
    ROS_ERROR("Face classifier unset.\n");
    ROS_WARN("Set a face classifier in ../launch/face_tracking.launch!\n");
    fail = true;
  }

  if (eye_cascade.compare("ERROR") == 0) {
    ROS_ERROR("Eye classifier unset.\n");
    ROS_WARN("Set an eye classifier in ../launch/face_tracking.launch!\n");
    fail = true;
  }

  if (smile_cascade.compare("ERROR") == 0) {
    ROS_ERROR("Smile classifier unset.\n");
    ROS_WARN("Set a smile classifier in ../launch/face_tracking.launch!\n");
    fail = true;
  }

  if (fail == true) {
    ROS_ERROR("Parameter loading failed. Shutting down...");
    ros::shutdown();
    exit(0);
  }
}

/**
 * @brief       Sets up ROS publishers and subscribers
 * @details     Sets publisher and subscriber names and sets up data publishers
 */

void FaceTracker::rosSetup() {
  sub_name = "/marty/camera/image";
  face_pub_name = "/marty/face_tracking/image/faces";
  eye_pub_name = "/marty/face_tracking/image/eyes";
  smile_pub_name = "/marty/face_tracking/image/smiles";

  faces_centroid_name = "/marty/face_tracking/faces_centroid";
  smiles_centroid_name = "/marty/face_tracking/smiles_centroid";
  eyes_centroid_name = "/marty/face_tracking/eyes_centroid";

  image_sub_ = it_.subscribe(sub_name, 1, &FaceTracker::imageCb, this);
  face_pub_ = it_.advertise(face_pub_name, 1);
  eye_pub_ = it_.advertise(eye_pub_name, 1);
  smile_pub_ = it_.advertise(smile_pub_name, 1);

  faces_centroid_pub_ =
      nh_.advertise<marty_msgs::CentroidMsg>(faces_centroid_name, 1);
  smiles_centroid_pub_ =
      nh_.advertise<marty_msgs::CentroidMsg>(smiles_centroid_name, 1);
  eyes_centroid_pub_ =
      nh_.advertise<marty_msgs::CentroidMsg>(eyes_centroid_name, 1);

  ROS_INFO_STREAM("Subscribing from: " << sub_name << std::endl);
  if (video_output == true) {
    ROS_INFO_STREAM("Publishing video to: " << face_pub_name << std::endl);
    ROS_INFO_STREAM("Publishing video to: " << smile_pub_name << std::endl);
    ROS_INFO_STREAM("Publishing video to: " << eye_pub_name << std::endl);
  }
  ROS_INFO_STREAM("Publishing co-ordinates to: " << faces_centroid_name
                                                 << std::endl);
  ROS_INFO_STREAM("Publishing co-ordinates to: " << smiles_centroid_name
                                                 << std::endl);
  ROS_INFO_STREAM("Publishing co-ordinates to: " << eyes_centroid_name
                                                 << std::endl);
}

/**
 * @brief       Load face, eye and smile classifiers
 * @details     Loads the classifiers as set by ROS params. Will shutdown if loading failed.
 */

void FaceTracker::loadClassifiers() {
  bool fail = false;
  bool loaded_face = face_classifier.load(face_cascade);
  bool loaded_eye = eye_classifier.load(eye_cascade);
  bool loaded_smile = smile_classifier.load(smile_cascade);

  if (loaded_face == false) {
    ROS_ERROR("Failed loading face classifier.\n");
    ROS_WARN("Does the file exist in the classifiers folder?\n");
    fail = true;
  }

  if (loaded_eye == false) {
    ROS_ERROR("Failed loading eye classifier.\n");
    ROS_WARN("Does the file exist in the classifiers folder?\n");
    fail = true;
  }

  if (loaded_smile == false) {
    ROS_ERROR("Failed loading smile classifier.\n");
    ROS_WARN("Does the file exist in the classifiers folder?\n");
    fail = true;
  }

  if (fail == true) {
    ROS_ERROR("Classifier loading failed. Shutting down...");
    ros::shutdown();
    exit(0);
  }
}

/**
 * @brief       Call back for an incoming image message. Handles image processing.
 * @details     Performs image processing, and applies a Cascade Classifier over these
 *              series of images to detect faces etc. Publishes image data and object
 *              co-ordinates.
 */

void FaceTracker::imageCb(const sensor_msgs::ImageConstPtr &msg) {

  try {
    // Converts ROS images to OpenCV
    face_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    face_region = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    eye_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    smile_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    // Converts the image color encoding to greyscale
    cv::cvtColor(face_image->image, grey_image, cv::COLOR_BGR2GRAY);
    // Equalizes the histogram of the grey image
    cv::equalizeHist(grey_image, grey_image);

  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // Calls the detection function and places facial co-ordinates within 'faces', a vector
  // of cv::Rect. Note: co-ordinates start from the top left of image, with y-axis vertically
  // downward.
  detectFaces(faces, grey_image);
  int faces_detected = faces.size();

  // Region of interest is the area of the image containing a face
  cv::Rect region_of_interest;
  // Offset is needed to remap the co-ordinates from ROI space to output space
  cv::Point offset[faces_detected];

  // Clears the co-ordinate containers and resizes them as needed
  resetCentroidMsg(faces_centroid, faces_detected);
  resetCentroidMsg(smiles_centroid, faces_detected);
  resetCentroidMsg(eyes_centroid, faces_detected);

  // For every face in the image...
  for (size_t i = 0; i < faces_detected; i++) {
    // ...draw a Rect over each face
    cv::rectangle(face_image->image, faces[i], cv::Scalar(255, 0, 0), 2);

    // ...if a valid co-ordinate
    if (faces[i].x > 0 && faces[i].y > 0) {
      // ...define ROI, area of this detected face
      region_of_interest =
          cv::Rect(faces[i].x, faces[i].y, faces[i].width, faces[i].height);

      // ...this face's centroid co-ordinates
      faces_centroid.x[i] = (int)faces[i].x + faces[i].width / 2;
      faces_centroid.y[i] = (int)faces[i].y + faces[i].height / 2;
      faces_centroid.z[i] = 0;

      // ...this face's origin co-ordinates in the output space (needed for smile and eyes)
      offset[i].x = faces[i].x;
      offset[i].y = faces[i].y;

      // ...crops the full size image to the ROI
      face_region->image = face_image->image(region_of_interest);
      // ...passes ROI to functions that detect eyes and smiles
      detectEyes(eyes, face_region->image);
      detectSmile(smiles, face_region->image);
    } else {
      // If co-ords aren't valid, just output an unmodified image
      face_region->image = face_image->image;
    }
  }

  // For each eye, calculate centroid and draw a rectangle over them
  for (size_t i = 0; i < eyes.size(); i++) {
    eyes[i] += offset[i];
    eyes_centroid.x[i] = (int)eyes[i].x + eyes[i].width / 2;
    eyes_centroid.y[i] = (int)eyes[i].y + eyes[i].height / 2;
    eyes_centroid.z[i] = 0;
    cv::rectangle(eye_image->image, eyes[i], cv::Scalar(0, 255, 0), 2);
  }

  // For each smile, calculate centroid and draw a rectangle over them
  for (size_t i = 0; i < smiles.size(); i++) {
    smiles[i] += offset[i];
    smiles_centroid.x[i] = (int)smiles[i].x + smiles[i].width / 2;
    smiles_centroid.y[i] = (int)smiles[i].y + smiles[i].height / 2;
    smiles_centroid.z[i] = 0;
    cv::rectangle(smile_image->image, smiles[i], cv::Scalar(0, 0, 255), 2);
  }

  // Publish data
  publishData(video_output);
}

/**
 * @brief       Publishes image data and detected object co-ordinates
 * @details     Always publishes object co-ordinates. Argument determines whether video is
 *              published.
 * @param       image True == video published, false == no video
 */

void FaceTracker::publishData(bool image) {

  if (image == true) {
    // Output modified video stream
    face_pub_.publish(face_image->toImageMsg());
    eye_pub_.publish(eye_image->toImageMsg());
    smile_pub_.publish(smile_image->toImageMsg());
  }

  // Publish centroid co-ordinates
  faces_centroid_pub_.publish(faces_centroid);
  smiles_centroid_pub_.publish(smiles_centroid);
  eyes_centroid_pub_.publish(eyes_centroid);
}

/**
 * @brief       Zeroes a CentroidMsg object
 * @details     Resizes the object to a given size and zeroes the internal arrays
 * @param       &msg CentroidMsg object to be modified
 * @param       size Size to resize the float arrays within a CentroidMsg
 */

void FaceTracker::resetCentroidMsg(marty_msgs::CentroidMsg &msg, int size) {
  msg.x.resize(size, 0);
  msg.y.resize(size, 0);
  msg.z.resize(size, 0);
}

/**
 * @brief       Applies a face detection CascadeClassifier to an input image
 * @details     Performs face detection on the given image. Saves the co-ordinates to a
 *              vector of cv::Rect objects. The detection parameters can be modified in the
 *              config file.
 * @param       &faces_vector Contains the co-ordinates of detected faces in the image
 * @param       image Desired image to apply face detection to
 */

void FaceTracker::detectFaces(std::vector<cv::Rect> &faces_vector,
                              cv::Mat image) {
  face_classifier.detectMultiScale(
      image, faces_vector, detection_parameters[0][0],
      detection_parameters[0][1], 0 | cv::CASCADE_SCALE_IMAGE,
      cv::Size(detection_parameters[0][2], detection_parameters[0][3]));
}

/**
 * @brief       Applies an eye detection CascadeClassifier to an input image
 * @details     Performs eye detection on the given image. Saves the co-ordinates to a
 *              vector of cv::Rect objects. The detection parameters can be modified in the
 *              config file.
 * @param       &eyes_vector Contains the co-ordinates of detected eyes in the image
 * @param       roi Desired image to apply face detection to
 */

void FaceTracker::detectEyes(std::vector<cv::Rect> &eyes_vector, cv::Mat roi) {
  eye_classifier.detectMultiScale(
      roi, eyes_vector, detection_parameters[1][0], detection_parameters[1][1],
      0 | cv::CASCADE_SCALE_IMAGE,
      cv::Size(detection_parameters[1][2], detection_parameters[1][3]));
}

/**
 * @brief       Applies a smile detection CascadeClassifier to an input image
 * @details     Performs smile detection on the given image. Saves the co-ordinates to a
 *              vector of cv::Rect objects. The detection parameters can be modified in the
 *              config file.
 * @param       &smiles_vector Contains the co-ordinates of detected smiles in the image
 * @param       roi Desired image to apply face detection to
 */

void FaceTracker::detectSmile(std::vector<cv::Rect> &smiles_vector, cv::Mat roi) {
  smile_classifier.detectMultiScale(
      roi, smiles_vector, detection_parameters[2][0], detection_parameters[2][1],
      0 | cv::CASCADE_SCALE_IMAGE,
      cv::Size(detection_parameters[2][2], detection_parameters[2][3]));
}

/**
 * @brief       Starts everything
 */

int main(int argc, char **argv) {
  ros::init(argc, argv, "face_tracking");

  ros::NodeHandle nh("");

  FaceTracker face_tracker(nh);

  ros::spin();
  return 0;
}
