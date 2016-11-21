#ifndef TRACKER_INCLUDE_TRACKER_TRACKER_HPP_
#define TRACKER_INCLUDE_TRACKER_TRACKER_HPP_

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include "dsst_tracker.hpp"
#include "tracker_run.hpp"

class DsstTrackerRun : public TrackerRun
{
public:
  DsstTrackerRun() :
      TrackerRun("DSSTcpp")
  {
  }

  virtual ~DsstTrackerRun()
  {
  }

  virtual cf_tracking::CfTracker* parseTrackerParas()
  {
    cf_tracking::DsstParameters paras;

    paras.padding = static_cast<double>(1);
    paras.outputSigmaFactor = static_cast<double>(1.0 / 16.0);
    paras.lambda = static_cast<double>(0.01);
    paras.learningRate = static_cast<double>(0.025);
    paras.templateSize = 100;
    paras.cellSize = 1;

    paras.enableTrackingLossDetection = false;
    paras.psrThreshold = 0;
    paras.psrPeakDel = 1;

    paras.enableScaleEstimator = true;
    paras.scaleSigmaFactor = static_cast<double>(0.25);
    paras.scaleStep = static_cast<double>(1.02);
    paras.scaleCellSize = 4;
    paras.numberOfScales = 33;

    paras.originalVersion = true;
    paras.resizeType = cv::INTER_AREA;

#ifdef DEBUG_TRACKER
    setTrackerDebug(&_debug);
    return new cf_tracking::DsstTracker(paras, &_debug);
#endif

    return new cf_tracking::DsstTracker(paras);
  }

private:
  cf_tracking::DsstDebug<cf_tracking::DsstTracker::T> _debug;
};

class Tracking
{
private:
  DsstTrackerRun mainObj;
public:
  Tracking();
  void callback(const sensor_msgs::ImageConstPtr& msg);
};

#endif /* TRACKER_INCLUDE_TRACKER_TRACKER_HPP_ */
