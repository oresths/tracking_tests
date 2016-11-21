#include "tracker/tracker.hpp"

Tracking::Tracking() {
  mainObj.init();
}

void Tracking::callback(const sensor_msgs::ImageConstPtr& msg) {
  ROS_DEBUG("I heard: []");

  try
  {
    mainObj.update(cv_bridge::toCvShare(msg, "bgr8")->image);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tracker");

  ros::NodeHandle nh;

  Tracking dsst;

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/camera/image_raw", 300, &Tracking::callback, &dsst);

  ros::spin();

  return 0;
}
