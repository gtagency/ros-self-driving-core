
#ifndef __LANE_EXTRACT_CV_H
#define __LANE_EXTRACT_CV_H

#include "cv.h"
#include "lane.h"
#include "ground_transform.h"

namespace ld {

    class LaneExtractCv {
    private:
        std::vector<Lane> lanes;
        cv::Mat processed;

        std::vector<std::pair<cv::Point, int> > findLanes(const cv::Mat& img, int boxHeight, int boxWidth); 
        Lane extractLane(const cv::Mat& img, cv::Point initialPoint, int boxHeight, int boxWidth); 
        void annotateImage(cv::Mat& img, const Lane& lane);
        void doLaneExtraction(const cv::Mat& src, const GroundTransform& gtrans, int maxLanes);

    
    public:
        LaneExtractCv(const cv::Mat& src, const GroundTransform& gtrans, int maxLanes);
        ~LaneExtractCv();

        const std::vector<Lane>& getLanes() const;

        const cv::Mat& getProcessedImage();
        int getProcessedImageEnc();
    };
}

#endif //__LANE_EXTRACT_CV_H
