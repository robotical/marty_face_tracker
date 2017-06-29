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
  FaceTracker::loadClassifiers();
  this->rosSetup();
}

FaceTracker::~FaceTracker() {}

void FaceTracker::loadParams() {
  bool fail = false;
  nh_.param<std::string>("face_classifier", face_cascade, "ERROR");
  nh_.param<std::string>("eye_classifier", eye_cascade, "ERROR");
  nh_.param<std::string>("smile_classifier", smile_cascade, "ERROR");

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

  face_centroid_name = "/marty/face_tracking/faces_centroid";

  image_sub_ = it_.subscribe(sub_name, 1, &FaceTracker::imageCb, this);
  face_pub_ = it_.advertise(face_pub_name, 1);
  eye_pub_ = it_.advertise(eye_pub_name, 1);
  smile_pub_ = it_.advertise(smile_pub_name, 1);

  face_centroid_ =
      nh_.advertise<marty_msgs::CentroidMsg>(face_centroid_name, 1);

  // face_centroid = it_.advertise(face_centroid_name, 1);

  ROS_INFO_STREAM("Subscribing from: " << sub_name << std::endl);
  ROS_INFO_STREAM("Publishing to: " << face_pub_name << std::endl);
  ROS_INFO_STREAM("Publishing to: " << smile_pub_name << std::endl);
  ROS_INFO_STREAM("Publishing to: " << eye_pub_name << std::endl);
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

  detect();
  int facesDetected = faces.size();

  cv::Rect region_of_interest;
  cv::Point offset;
  marty_msgs::CentroidMsg centroid;
  centroid.x.resize(facesDetected);
  centroid.y.resize(facesDetected);
  centroid.z.resize(facesDetected);

  for (size_t i = 0; i < facesDetected; i++) {
    cv::rectangle(face_image->image, faces[i], cv::Scalar(255, 0, 0), 2);

    if (faces[i].x > 0 && faces[i].y > 0) {
      region_of_interest =
          cv::Rect(faces[i].x, faces[i].y, faces[i].width, faces[i].height);

      centroid.x[i] = (int)faces[i].x + faces[i].width / 2;
      centroid.y[i] = (int)faces[i].y + faces[i].height / 2;
      centroid.z[i] = 0;

      offset.x = faces[i].x;
      offset.y = faces[i].y;
      face_region->image = face_image->image(region_of_interest);
      detectEyes(face_region->image);
      detectSmile(face_region->image);
    } else {
      face_region->image = face_image->image;
    }
  }

  for (size_t i = 0; i < facesDetected; i++) {
  }

  for (size_t i = 0; i < eyes.size(); i++) {
    eyes[i] += offset;
    cv::rectangle(eye_image->image, eyes[i], cv::Scalar(0, 255, 0), 2);
  }

  for (size_t i = 0; i < smiles.size(); i++) {
    std::cout << "detected " << smiles.size() << " smiles" << std::endl;
    smiles[i] += offset;
    std::cout << "smile " << i << " co-oords " << smiles[i] << std::endl;
    cv::rectangle(smile_image->image, smiles[i], cv::Scalar(0, 0, 255), 2);
  }

  cv::waitKey(3);
  // Output modified video stream
  face_pub_.publish(face_image->toImageMsg());
  eye_pub_.publish(eye_image->toImageMsg());
  smile_pub_.publish(smile_image->toImageMsg());
  face_centroid_.publish(centroid);
}

void FaceTracker::detect() {
  face_classifier.detectMultiScale(
      grey_image, faces, 1.1, 3, 0 | cv::CASCADE_SCALE_IMAGE, cv::Size(30, 30));
}

void FaceTracker::detectEyes(cv::Mat roi) {
  eye_classifier.detectMultiScale(
      roi, eyes, 1.1, 3, 0 | cv::CASCADE_SCALE_IMAGE, cv::Size(30, 30));
}

void FaceTracker::detectSmile(cv::Mat roi) {
  smile_classifier.detectMultiScale(
      roi, smiles, 1.1, 3, 0 | cv::CASCADE_SCALE_IMAGE, cv::Size(30, 30));
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "face_tracking");

  ros::NodeHandle nh("");

  FaceTracker face_tracker(nh);

  ros::spin();
  return 0;
}