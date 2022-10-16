//
// Created by lifu on 6/6/17.
//

#ifndef PROJECT_MAPFUSION_H
#define PROJECT_MAPFUSION_H

//#include <include/KeyFrame.h>

#include <queue>
#include <thread>

#include "corbslam_server/GlobalOptimize.h"
#include <corbslam_client/KeyFrame.h>
#include <corbslam_client/ORBmatcher.h>
#include <corbslam_client/PnPsolver.h>
#include "corbslam_server/PubToClient.h"
#include "corbslam_server/ServerMap.h"
#include <corbslam_client/TransPose.h>
#include <corbslam_msgs/corbslam_insert.h>
#include <corbslam_msgs/corbslam_message.h>
#include <corbslam_msgs/corbslam_update.h>
#include <ros/ros.h>

using namespace ORB_SLAM2;

namespace CORBSLAM_SERVER {

class MapFusion {
 public:
  // typedef std::function<bool(const size_t&, const double&, const size_t&,
  //                            const double&, const cv::Mat&, const cv::Mat&)>
  //     FusionPubFunc;

  using FusionPubFunc=GlobalOptimize::FusionPubFunc;

  MapFusion(std::string strSettingPath, FusionPubFunc fusionPubFunc=FusionPubFunc());

  void createSubServerMap(int mapId);

  void runPubTopic();

  bool insertKeyFrameToMap(corbslam_msgs::corbslam_insert::Request& req,
                           corbslam_msgs::corbslam_insert::Response& res);

  bool insertMapPointToMap(corbslam_msgs::corbslam_insert::Request& req,
                           corbslam_msgs::corbslam_insert::Response& res);

  bool updateKeyFrameToMap(corbslam_msgs::corbslam_update::Request& req,
                           corbslam_msgs::corbslam_update::Response& res);

  bool updateMapPointToMap(corbslam_msgs::corbslam_update::Request& req,
                           corbslam_msgs::corbslam_update::Response& res);

  void fuseSubMapToMap();

  bool loadORBVocabulary(const string& strVocFile);

  void createKeyFrameDatabase();

  bool mapFuse(ServerMap* sMapx, ServerMap* sMapy);

  bool mapFuseToGlobalMap(ServerMap* sMap);

  bool detectKeyFrameInServerMap(ServerMap* sMap, KeyFrame* tKF,
                                 cv::Mat& newPose,
                                 std::vector<KeyFrame*>& candidateKFs);

  void insertServerMapToGlobleMap(ServerMap* sMap, cv::Mat To2n);

 private:
  // ORB vocabulary used for place recognition and feature matching.
  ORBVocabulary* mpVocabulary;

  GlobalOptimize* mpGBA;

  std::string mpStrSettingPath;

  bool ifSubToGlobalMap[100];

  bool ifNullGlobalMap;

  bool resentGlobalMap;

  std::mutex resentGlobalMapMutex;
  std::mutex nullGlobalMapMutex;

  cv::Mat subMapTransM[100];

  // KeyFrame database for place recognition (relocalization and loop
  // detection).
  KeyFrameDatabase* mpKeyFrameDatabase;

  std::map<int, ServerMap*> serverMap;
  std::map<int, std::queue<long unsigned int>> mKfUpdated;
  std::queue<long unsigned int> mKfUpdatedGlobal;

  ServerMap* globalMap;

  std::mutex mStreamInOutPutMutex;

  std::mutex mSubMapUpdatedMutex;

  std::mutex mAddUpdatedKfMutex;

  inline void addUpdatedKf(int clientId, long unsigned int kfId) {
    std::lock_guard<std::mutex> addUpdatedKfMutex(mAddUpdatedKfMutex);
    if (!mKfUpdated.count(clientId)) {
      mKfUpdated.emplace(clientId, std::queue<long unsigned int>());
    }
    mKfUpdated[clientId].emplace(kfId);
  }
  inline void addUpdatedKf(long unsigned int kfId) {
    mKfUpdatedGlobal.emplace(kfId);
  }
  inline std::queue<long unsigned int> getUpdatedKf(int clientId) {
    std::lock_guard<std::mutex> addUpdatedKfMutex(mAddUpdatedKfMutex);
    if (mKfUpdated.count(clientId)) {
      return mKfUpdated[clientId];
    } else {
      return std::queue<long unsigned int>();
    }
  }
  inline bool getNextUpdatedKf(int clientId, KeyFrame** pKF) {
    std::lock_guard<std::mutex> addUpdatedKfMutex(mAddUpdatedKfMutex);
    if (serverMap.count(clientId) && mKfUpdated.count(clientId) &&
        mKfUpdated[clientId].size()) {
      *pKF = serverMap[clientId]->pCacher->getKeyFrameById(
          mKfUpdated[clientId].front());
      mKfUpdated[clientId].pop();
      return true;
    } else
      return false;
  }
  inline bool clearUpdatedKf(int clientId) {
    std::lock_guard<std::mutex> addUpdatedKfMutex(mAddUpdatedKfMutex);
    if (mKfUpdated.count(clientId)) {
      mKfUpdated[clientId] = std::queue<long unsigned int>();
    }
  }

  FusionPubFunc mfFusionPubFunc;
};
}  // namespace CORBSLAM_SERVER

#endif  // PROJECT_MAPFUSION_H
