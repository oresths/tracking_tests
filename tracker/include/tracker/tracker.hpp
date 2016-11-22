#ifndef TRACKER_INCLUDE_TRACKER_TRACKER_HPP_
#define TRACKER_INCLUDE_TRACKER_TRACKER_HPP_

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <dsst_tracker_run.hpp>

#include "dsst_tracker.hpp"
#include "tracker_run.hpp"
#include "dsst_tracker_run.hpp"

class Tracking
{
private:
  DsstTrackerRun mainObj;
public:
  Tracking();
  void callback(const sensor_msgs::ImageConstPtr& msg);
};

#endif /* TRACKER_INCLUDE_TRACKER_TRACKER_HPP_ */
