/**
 * @file      face_tracking.cpp
 * @brief     Marty tracks a face
 * @author    Helmi Fraser <helmi@robotical.io>
 * @date      2017-06-23
 * @copyright (Apache) 2016 Robotical Ltd.
 */

#include <marty_face_tracker/face_tracking.hpp>

FaceTracker::FaceTracker(ros::NodeHandle &nh) : it_(nh) {
  image_sub_ = it_.subscribe(sub_name, 1, &FaceTracker::imageCb, this);
  face_pub_ = it_.advertise(face_pub_name, 1);
  eye_pub_ = it_.advertise(eye_pub_name, 1);
  smile_pub_ = it_.advertise(smile_pub_name, 1);

  FaceTracker::loadClassifiers();
}

FaceTracker::~FaceTracker() { cv::destroyWindow(FACE_DETECT_OUT); }

// TODO: make this dynamic and not hardcoded

void FaceTracker::loadClassifiers() {
  bool loaded_face = face_classifier.load(face_cascade);
  bool loaded_eye = eye_classifier.load(eye_cascade);
  bool loaded_smile = smile_classifier.load(smile_cascade);
}

void FaceTracker::imageCb(const sensor_msgs::ImageConstPtr &msg) {
  cv_bridge::CvImagePtr cv_ptr, eye_image, smile_image;
  cv::Mat grey_image;
  std::vector<cv::Rect> faces;
  std::vector<cv::Rect> eyes;
  std::vector<cv::Rect> smiles;

  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    eye_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    smile_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    cv::cvtColor(cv_ptr->image, grey_image, cv::COLOR_BGR2GRAY);
    cv::equalizeHist(grey_image, grey_image);

  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

// TODO: experiment dealing with noise and improve smile detection, it's atrocious

  face_classifier.detectMultiScale(
      grey_image, faces, 1.1, 3, 0 | cv::CASCADE_SCALE_IMAGE, cv::Size(30, 30));
  eye_classifier.detectMultiScale(
      grey_image, eyes, 1.1, 3, 0 | cv::CASCADE_SCALE_IMAGE, cv::Size(30, 30));
  smile_classifier.detectMultiScale(grey_image, smiles, 1.1, 3,
                                    0 | cv::CASCADE_SCALE_IMAGE,
                                    cv::Size(40, 40));

  for (size_t i = 0; i < faces.size(); i++) {
    cv::rectangle(cv_ptr->image, faces[i], cv::Scalar(255, 0, 0), 2);
  }

  for (size_t i = 0; i < eyes.size(); i++) {
    cv::rectangle(eye_image->image, eyes[i], cv::Scalar(0, 255, 0), 2);
  }

  for (size_t i = 0; i < smiles.size(); i++) {
    cv::rectangle(smile_image->image, smiles[i], cv::Scalar(0, 0, 255), 2);
  }

  cv::waitKey(3);

  // Output modified video stream
  face_pub_.publish(cv_ptr->toImageMsg());
  eye_pub_.publish(eye_image->toImageMsg());
  smile_pub_.publish(smile_image->toImageMsg());

}

int main(int argc, char **argv) {
  ros::init(argc, argv, "face_tracking");

  ros::NodeHandle nh("");

  FaceTracker ic(nh);

  // std::cout << "Subscribing from: " << sub_name << std::endl;
  // std::cout << "Publishing to: " << pub_name << std::endl;

  ros::spin();
  return 0;
}
