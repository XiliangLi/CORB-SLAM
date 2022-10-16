//
// Created by lifu on 2017/6/4.
//

#include <corbslam_msgs/MapFusion.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <opencv2/core/core.hpp>

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <thread>

#include "corbslam_server/MapFusion.h"
#include "corbslam_server/ServerMap.h"

class FusionPublisher {
 public:
  FusionPublisher(const ros::NodeHandle& nh) : nh_(nh) {
    map_fusion_pub_ = nh_.advertise<corbslam_msgs::MapFusion>(
        "map_fusion_out", 10, true);
  }
  ~FusionPublisher() = default;

  bool publishFusion(const size_t& from_client_id, const double& from_timestamp,
                     const size_t& to_client_id, const double& to_timestamp,
                     const cv::Mat& R, const cv::Mat& t) {
    tf2::Matrix3x3 tf2_rot(
        R.at<float>(0, 0), R.at<float>(0, 1), R.at<float>(0, 2),
        R.at<float>(1, 0), R.at<float>(1, 1), R.at<float>(1, 2),
        R.at<float>(2, 0), R.at<float>(2, 1), R.at<float>(2, 2));
    tf2::Quaternion tf2_quaternion;
    tf2_rot.getRotation(tf2_quaternion);

    corbslam_msgs::MapFusion map_fusion_msg;
    map_fusion_msg.from_client_id = from_client_id;
    map_fusion_msg.from_timestamp = ros::Time(from_timestamp);
    map_fusion_msg.to_client_id = to_client_id;
    map_fusion_msg.to_timestamp = ros::Time(to_timestamp);
    map_fusion_msg.transform.rotation = tf2::toMsg(tf2_quaternion);
    map_fusion_msg.transform.translation.x = t.at<float>(0);
    map_fusion_msg.transform.translation.y = t.at<float>(1);
    map_fusion_msg.transform.translation.z = t.at<float>(2);
    map_fusion_pub_.publish(map_fusion_msg);
    ROS_INFO(
        "Map Fusion Message Published, from client %d time %d, to client "
        "%d time %d ",
        from_client_id, map_fusion_msg.from_timestamp, to_client_id,
        map_fusion_msg.to_timestamp);

    return true;
  }

 private:
  ros::NodeHandle nh_;
  ros::Publisher map_fusion_pub_;
};

using namespace std;

using namespace CORBSLAM_SERVER;

int main(int argc, char** argv) {
  ros::init(argc, argv, "Corbslam_server");
  ros::start();

  if (argc != 3) {
    cout << "lack of vocabulary path !!!\n";
    return false;
  }
  ROS_INFO("Corbslam_server start!");

  // Create MapFusion system. It initializes all system threads and gets ready
  // to process frames.

  std::string strSettingPath = argv[2];

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  FusionPublisher fusion_pub(nh);

  MapFusion* mapfusion = new MapFusion(
      strSettingPath, boost::bind(&FusionPublisher::publishFusion, &fusion_pub,
                                  _1, _2, _3, _4, _5, _6));

  mapfusion->loadORBVocabulary(argv[1]);

  mapfusion->createKeyFrameDatabase();

  ros::NodeHandle n;

  ros::ServiceServer InsertKeyFrameService = n.advertiseService(
      "insertKeyFrameToMap", &MapFusion::insertKeyFrameToMap, mapfusion);

  ros::ServiceServer InsertMapPointService = n.advertiseService(
      "insertMapPointToMap", &MapFusion::insertMapPointToMap, mapfusion);

  ros::ServiceServer updateKeyFrameToMapService = n.advertiseService(
      "updateKeyFrameToMap", &MapFusion::updateKeyFrameToMap, mapfusion);

  ros::ServiceServer updateMapPointToMapService = n.advertiseService(
      "updateMapPointToMap", &MapFusion::updateMapPointToMap, mapfusion);

  ROS_INFO("Publish services finished !");

  std::thread* mapFuisonThread =
      new thread(&MapFusion::fuseSubMapToMap, mapfusion);

  // publish update keyframe and mappoint poses topic
  std::thread* pubThread = new thread(&MapFusion::runPubTopic, mapfusion);

  // wait to get subcribe new keyframes or new mappoints
  ros::MultiThreadedSpinner spinner(2);

  spinner.spin();

  // Stop all threads

  // Save camera trajectory

  ros::shutdown();

  return 0;
}
