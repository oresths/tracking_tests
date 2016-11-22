

#ifndef CF_TRACKING_SRC_MAIN_DSST_INTERFACE_HPP_
#define CF_TRACKING_SRC_MAIN_DSST_INTERFACE_HPP_

#include "dsst_tracker_run.hpp"

class DsstInterface
{
private:
  DsstTrackerRun mainObj;
public:
  DsstInterface() { mainObj.init(); }
  void dsstUpdate(cv::Mat im) { mainObj.update(im); }
};


#endif /* CF_TRACKING_SRC_MAIN_DSST_INTERFACE_HPP_ */
