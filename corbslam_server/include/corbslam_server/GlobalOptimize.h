//
// Created by lifu on 6/29/17.
//

#ifndef PROJECT_GLOBALOPTIMIZE_H
#define PROJECT_GLOBALOPTIMIZE_H

#include <g2o/types/types_seven_dof_expmap.h>

#include <corbslam_client/Cache.h>
#include "corbslam_server/ServerMap.h"

#include<mutex>

#include <time.h>

using namespace ORB_SLAM2;

namespace CORBSLAM_SERVER {

    class GlobalOptimize {

    public:

        typedef pair<set<KeyFrame*>,int> ConsistentGroup;
        typedef map<KeyFrame*,g2o::Sim3,std::less<KeyFrame*>,
                Eigen::aligned_allocator<std::pair<const KeyFrame*, g2o::Sim3> > > KeyFrameAndPose;

        typedef boost::function<bool(const size_t&, const double&, const size_t&,
                             const double&, const cv::Mat&, const cv::Mat&)>
                FusionPubFunc;

    public:

        GlobalOptimize(ServerMap * tgm, FusionPubFunc fusionPubFunc=FusionPubFunc());

        void setCurentKeyFrame( KeyFrame * pCurKF);

        void setCandidates( std::vector<KeyFrame*> pCandites);

        // This function will run in a separate thread
        void RunGlobalBundleAdjustment(unsigned long nLoopKF);

        bool ComputeSim3();

        void SearchAndFuse(const KeyFrameAndPose &CorrectedPosesMap);

        void CorrectLoop();

        bool isRunningGBA(){
            unique_lock<std::mutex> lock(mMutexGBA);
            return mbRunningGBA;
        }
        bool isFinishedGBA(){
            unique_lock<std::mutex> lock(mMutexGBA);
            return mbFinishedGBA;
        }

    protected:
        ServerMap * globalMAp;

        Cache* mpCacher;

        std::list<KeyFrame*> mlpLoopKeyFrameQueue;

        std::mutex mMutexLoopQueue;

        // Loop detector parameters
        float mnCovisibilityConsistencyTh;

        // Loop detector variables
        KeyFrame* mpCurrentKF;
        KeyFrame* mpMatchedKF;
        std::vector<ConsistentGroup> mvConsistentGroups;
        std::vector<KeyFrame*> mvpEnoughConsistentCandidates;
        std::vector<KeyFrame*> mvpCurrentConnectedKFs;
        std::vector<MapPoint*> mvpCurrentMatchedPoints;
        std::vector<MapPoint*> mvpLoopMapPoints;
        cv::Mat mScw;
        g2o::Sim3 mg2oScw;

        long unsigned int mLastLoopKFid;

        // Variables related to Global Bundle Adjustment
        bool mbRunningGBA;
        bool mbFinishedGBA;
        bool mbStopGBA;
        std::mutex mMutexGBA;
        std::thread* mpThreadGBA;

        // Fix scale in the stereo/RGB-D case
        bool mbFixScale;

        bool mnFullBAIdx;

        FusionPubFunc mfFusionPubFunc;
    };

}


#endif //PROJECT_GLOBALOPTIMIZE_H
