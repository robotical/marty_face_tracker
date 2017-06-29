/**
 * @file      face_tracking.cpp
 * @brief     Marty tracks a face
 * @author    Helmi Fraser <helmi@robotical.io>
 * @date      2017-06-23
 * @copyright (Apache) 2016 Robotical Ltd.
 */

#include <marty_face_tracker/face_tracking.hpp>

FaceTracker::FaceTracker(ros::NodeHandle &nh) : it_(nh) {
  this->loadParams();
  this->loadClassifiers();
  this->rosSetup();
}

FaceTracker::~FaceTracker() {}

void FaceTracker::loadParams() {
  bool fail = false;
  nh_.param<std::string>("face_classifier", face_cascade, "ERROR");
  nh_.param<std::string>("eye_classifier", eye_cascade, "ERROR");
  nh_.param<std::string>("smile_classifier", smile_cascade, "ERROR");

  nh_.param<bool>("video_output", video_output, false);

  if (video_output == true) {
    ROS_WARN("video_output set to true");
  } else {
    ROS_WARN("video_output set to false");
  }

  std::vector<std::string> param1 = {"face", "eye", "smile"};
  std::vector<std::string> param2 = {"_scale_factor", "_min_neighbours",
                                     "_min_size", "_max_size"};
  float default_vals[4] = {1.1, 3, 30, 30};
  bool default_param = true;
  nh_.param<bool>("ignore", default_param, true);

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

void FaceTracker::imageCb(const sensor_msgs::ImageConstPtr &msg) {

  try {
    face_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    face_region = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    eye_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    smile_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    cv::cvtColor(face_image->image, grey_image, cv::COLOR_BGR2GRAY);
    cv::equalizeHist(grey_image, grey_image);

  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  detectFaces(faces, grey_image);
  int facesDetected = faces.size();

  cv::Rect region_of_interest;
  cv::Point offset[facesDetected];

  resetCentroidMsg(faces_centroid, facesDetected);
  resetCentroidMsg(smiles_centroid, facesDetected);
  resetCentroidMsg(eyes_centroid, facesDetected);

  for (size_t i = 0; i < facesDetected; i++) {
    cv::rectangle(face_image->image, faces[i], cv::Scalar(255, 0, 0), 2);

    if (faces[i].x > 0 && faces[i].y > 0) {
      region_of_interest =
          cv::Rect(faces[i].x, faces[i].y, faces[i].width, faces[i].height);

      faces_centroid.x[i] = (int)faces[i].x + faces[i].width / 2;
      faces_centroid.y[i] = (int)faces[i].y + faces[i].height / 2;
      faces_centroid.z[i] = 0;

      offset[i].x = faces[i].x;
      offset[i].y = faces[i].y;

      face_region->image = face_image->image(region_of_interest);
      // face_region->image = grey_image(region_of_interest);
      detectEyes(face_region->image);
      detectSmile(face_region->image);
    } else {
      face_region->image = face_image->image;
    }
  }

  for (size_t i = 0; i < eyes.size(); i++) {
    eyes[i] += offset[i];
    eyes_centroid.x[i] = (int)eyes[i].x + eyes[i].width / 2;
    eyes_centroid.y[i] = (int)eyes[i].y + eyes[i].height / 2;
    eyes_centroid.z[i] = 0;
    cv::rectangle(eye_image->image, eyes[i], cv::Scalar(0, 255, 0), 2);
  }

  for (size_t i = 0; i < smiles.size(); i++) {
    smiles[i] += offset[i];
    smiles_centroid.x[i] = (int)smiles[i].x + smiles[i].width / 2;
    smiles_centroid.y[i] = (int)smiles[i].y + smiles[i].height / 2;
    smiles_centroid.z[i] = 0;
    cv::rectangle(smile_image->image, smiles[i], cv::Scalar(0, 0, 255), 2);
  }

  publishData(video_output);
}

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

void FaceTracker::resetCentroidMsg(marty_msgs::CentroidMsg &msg, int size) {
  msg.x.resize(size, 0);
  msg.y.resize(size, 0);
  msg.z.resize(size, 0);
}

void FaceTracker::detectFaces(std::vector<cv::Rect> &facesVector,
                              cv::Mat image) {
  face_classifier.detectMultiScale(
      image, facesVector, detection_parameters[0][0],
      detection_parameters[0][1], 0 | cv::CASCADE_SCALE_IMAGE,
      cv::Size(detection_parameters[0][2], detection_parameters[0][3]));
}

void FaceTracker::detectEyes(cv::Mat roi) {
  eye_classifier.detectMultiScale(
      roi, eyes, detection_parameters[1][0], detection_parameters[1][1],
      0 | cv::CASCADE_SCALE_IMAGE,
      cv::Size(detection_parameters[1][2], detection_parameters[1][3]));
}

void FaceTracker::detectSmile(cv::Mat roi) {
  smile_classifier.detectMultiScale(
      roi, smiles, detection_parameters[2][0], detection_parameters[2][1],
      0 | cv::CASCADE_SCALE_IMAGE,
      cv::Size(detection_parameters[2][2], detection_parameters[2][3]));
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "face_tracking");

  ros::NodeHandle nh("");

  FaceTracker face_tracker(nh);

  ros::spin();
  return 0;
}
