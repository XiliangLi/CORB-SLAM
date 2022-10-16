/**
 * This file is part of ORB-SLAM2.
 *
 * Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University
 * of Zaragoza) For more information see <https://github.com/raulmur/ORB_SLAM2>
 *
 * ORB-SLAM2 is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ORB-SLAM2 is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
 */

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <string>

#include <corbslam_msgs/LoopClosure.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <opencv2/core/core.hpp>

#include "corbslam_client/System.h"

using namespace std;

typedef boost::function<void(cv::Mat, double)> TfPubFunc;

class TfPublisher {
 public:
  TfPublisher(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,
              int client_id)
      : nh_(nh), nh_private_(nh_private), current_time_(ros::Time::now()) {
    tf_timer_ =
        nh_.createTimer(ros::Duration(0.01),
                        &TfPublisher::PublishPositionAsTransformCallback, this);
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odometry", 10, true);
    nh_private_.param<std::string>(
        "map_frame", map_frame_,
        "map_" + static_cast<std::string>(std::to_string(client_id)));
    nh_private_.param<std::string>(
        "camera_frame", camera_frame_,
        "robot_base_" + static_cast<std::string>(std::to_string(client_id)));
  }
  ~TfPublisher() = default;

  void updatePose(cv::Mat pose, double timestamp) {
    if (pose.empty()) return;
    std::lock_guard<std::mutex> pose_update_lock(pose_update_mutex_);
    current_position_ = TransformFromMat(pose);
    current_time_.fromSec(timestamp);
  }

  void PublishPositionAsTransformCallback(const ros::TimerEvent& event) {
    std::lock_guard<std::mutex> pose_update_lock(pose_update_mutex_);
    tf_broadcaster_.sendTransform(tf::StampedTransform(
        current_position_, current_time_, map_frame_, camera_frame_));
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = current_time_;
    odom_msg.header.frame_id = map_frame_;
    odom_msg.child_frame_id = camera_frame_;
    odom_msg.pose.pose.position.x = current_position_.getOrigin().x();
    odom_msg.pose.pose.position.y = current_position_.getOrigin().y();
    odom_msg.pose.pose.position.z = current_position_.getOrigin().z();
    odom_msg.pose.pose.orientation.w = current_position_.getRotation().w();
    odom_msg.pose.pose.orientation.x = current_position_.getRotation().x();
    odom_msg.pose.pose.orientation.y = current_position_.getRotation().y();
    odom_msg.pose.pose.orientation.z = current_position_.getRotation().z();
    odom_pub_.publish(odom_msg);
  }

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Time current_time_;
  tf::Transform current_position_;
  std::string map_frame_;
  std::string camera_frame_;

  tf::TransformBroadcaster tf_broadcaster_;
  ros::Publisher odom_pub_;

  ros::Timer tf_timer_;

  std::mutex pose_update_mutex_;

  tf::Transform TransformFromMat(cv::Mat position_mat) {
    cv::Mat rotation(3, 3, CV_32F);
    cv::Mat translation(3, 1, CV_32F);

    rotation = position_mat.rowRange(0, 3).colRange(0, 3);
    translation = position_mat.rowRange(0, 3).col(3);

    tf::Matrix3x3 tf_camera_rotation(
        rotation.at<float>(0, 0), rotation.at<float>(0, 1),
        rotation.at<float>(0, 2), rotation.at<float>(1, 0),
        rotation.at<float>(1, 1), rotation.at<float>(1, 2),
        rotation.at<float>(2, 0), rotation.at<float>(2, 1),
        rotation.at<float>(2, 2));

    tf::Vector3 tf_camera_translation(translation.at<float>(0),
                                      translation.at<float>(1),
                                      translation.at<float>(2));

    // Coordinate transformation matrix from orb coordinate system to ros
    // coordinate system
    const tf::Matrix3x3 tf_orb_to_ros(0, 0, 1, -1, 0, 0, 0, -1, 0);

    // Transform from orb coordinate system to ros coordinate system on camera
    // coordinates
    tf_camera_rotation = tf_orb_to_ros * tf_camera_rotation;
    tf_camera_translation = tf_orb_to_ros * tf_camera_translation;

    // Inverse matrix
    tf_camera_rotation = tf_camera_rotation.transpose();
    tf_camera_translation = -(tf_camera_rotation * tf_camera_translation);

    // Transform from orb coordinate system to ros coordinate system on map
    // coordinates
    tf_camera_rotation = tf_orb_to_ros * tf_camera_rotation;
    tf_camera_translation = tf_orb_to_ros * tf_camera_translation;

    return tf::Transform(tf_camera_rotation, tf_camera_translation);
  }
};

class LoopPublisher {
 public:
  LoopPublisher(const ros::NodeHandle& nh, int client_id)
      : nh_(nh), client_id_(client_id) {
    loop_closure_pub_ =
        nh_.advertise<corbslam_msgs::LoopClosure>("loop_closure_out", 10, true);
  }
  ~LoopPublisher() = default;

  bool publishLoop(const double& from_timestamp, const double& to_timestamp,
                   const cv::Mat& R, const cv::Mat& t) {
    tf2::Matrix3x3 tf2_rot(
        R.at<float>(0, 0), R.at<float>(0, 1), R.at<float>(0, 2),
        R.at<float>(1, 0), R.at<float>(1, 1), R.at<float>(1, 2),
        R.at<float>(2, 0), R.at<float>(2, 1), R.at<float>(2, 2));
    tf2::Quaternion tf2_quaternion;
    tf2_rot.getRotation(tf2_quaternion);

    corbslam_msgs::LoopClosure loop_closure_msg;
    loop_closure_msg.from_timestamp = ros::Time(from_timestamp);
    loop_closure_msg.to_timestamp = ros::Time(to_timestamp);
    loop_closure_msg.transform.rotation = tf2::toMsg(tf2_quaternion);
    loop_closure_msg.transform.translation.x = t.at<float>(0);
    loop_closure_msg.transform.translation.y = t.at<float>(1);
    loop_closure_msg.transform.translation.z = t.at<float>(2);
    loop_closure_pub_.publish(loop_closure_msg);
    ROS_INFO(
        "Loop Closure Message Published, from client %d time %d, to "
        "time %d",
        client_id_, loop_closure_msg.from_timestamp,
        loop_closure_msg.to_timestamp);

    return true;
  }

 private:
  ros::NodeHandle nh_;
  ros::Publisher loop_closure_pub_;

  int client_id_;
};

class ImageGrabber {
 public:
  ImageGrabber(ORB_SLAM2::System* pSLAM, TfPubFunc tf_pub_func)
      : mpSLAM(pSLAM), tf_pub_func_(tf_pub_func) {}

  void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,
                const sensor_msgs::ImageConstPtr& msgD);

  ORB_SLAM2::System* mpSLAM;

  TfPubFunc tf_pub_func_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "RGBD");
  ros::start();

  if (argc != 4) {
    cerr << endl
         << "Usage: rosrun ORB_SLAM2 RGBD path_to_vocabulary path_to_settings "
            "client_id"
         << endl;
    ros::shutdown();
    return 1;
  }

  int clientId = boost::lexical_cast<int>(argv[3]);

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  TfPublisher tf_pub_functor(nh, nh_private, clientId);
  LoopPublisher loop_pub(nh, clientId);

  // Create SLAM system. It initializes all system threads and gets ready to
  // process frames.
  ORB_SLAM2::System SLAM(
      argv[1], argv[2], ORB_SLAM2::System::RGBD, false, clientId,
      boost::bind(&LoopPublisher::publishLoop, &loop_pub, _1, _2, _3, _4));

  ImageGrabber igb(
      &SLAM, boost::bind(&TfPublisher::updatePose, &tf_pub_functor, _1, _2));

  message_filters::Subscriber<sensor_msgs::Image> rgb_sub(
      nh, "camera/rgb/image_raw", 1);
  message_filters::Subscriber<sensor_msgs::Image> depth_sub(
      nh, "camera/depth_registered/image_raw", 1);
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                          sensor_msgs::Image>
      sync_pol;
  message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,
                                               depth_sub);
  sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD, &igb, _1, _2));

  ros::spin();

  // Stop all threads
  SLAM.Shutdown();

  // Save camera trajectory
  SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

  ros::shutdown();

  return 0;
}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,
                            const sensor_msgs::ImageConstPtr& msgD) {
  // Copy the ros image message to cv::Mat.
  cv_bridge::CvImageConstPtr cv_ptrRGB;
  try {
    cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  cv_bridge::CvImageConstPtr cv_ptrD;
  try {
    cv_ptrD = cv_bridge::toCvShare(msgD);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  tf_pub_func_(mpSLAM->TrackRGBD(cv_ptrRGB->image, cv_ptrD->image,
                                 cv_ptrRGB->header.stamp.toSec()),
               cv_ptrRGB->header.stamp.toSec());
}
