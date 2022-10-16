//
// Created by lifu on 6/8/17.
//

#ifndef PROJECT_SERVERMAP_H
#define PROJECT_SERVERMAP_H

#include <corbslam_client/KeyFrame.h>
#include <corbslam_client/MapPoint.h>
#include <corbslam_client/Cache.h>
#include <corbslam_client/Map.h>


using namespace ORB_SLAM2;

namespace CORBSLAM_SERVER{


    class ServerMap {

    public:

        ServerMap();

        ServerMap( Cache * tCache, Map * tMap );

        std::vector<KeyFrame*> DetectMapFusionCandidatesFromDB(KeyFrame * tKF);

        void clear();

    public:

        Cache * pCacher;

        Map * pMap;

    };
}


#endif //PROJECT_SERVERMAP_H
