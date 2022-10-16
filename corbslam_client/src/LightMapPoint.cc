//
// Created by lifu on 17-2-2.
//


#include "corbslam_client/System.h"
#include "corbslam_client/Cache.h"
#include "corbslam_client/LightKeyFrame.h"
#include "corbslam_client/Converter.h"
#include <thread>
#include <iomanip>
#include "corbslam_client/LightMapPoint.h"


namespace ORB_SLAM2 {

    LightMapPoint::LightMapPoint() {
        this->mnMapPointId = 0;
        this->mpCache = nullptr;
    }
    LightMapPoint::LightMapPoint(MapPoint *pMP) {
        if( pMP) {
            this->mnMapPointId = pMP->mnId;
            this->mpCache = pMP->getCache();
        }else {
            this->mnMapPointId = 0;
            this->mpCache = nullptr;
        }

    }
    LightMapPoint::LightMapPoint(long unsigned int pId, Cache *pCache) {

        this->mnMapPointId = pId;
        this->mpCache = pCache;

    }
    MapPoint* LightMapPoint::getMapPoint() const {

        if( this->mpCache )
            return this->mpCache->getMapPointById( this->mnMapPointId);
        else
            return nullptr;

    }


}
