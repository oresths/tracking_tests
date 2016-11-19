#include "ros/ros.h"
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

int main(int argc, char **argv)
{

  ros::init(argc, argv, "stream_n_track");

  ros::NodeHandle nh;

  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("/camera/image_raw", 1);


  cv::VideoCapture cap("/mnt/hgfs/Vision/Dataset UAV123/UAV123/data_seq/UAV123/bike1/%06d.jpg");
  // Check if video device can be opened with the given index
  if(!cap.isOpened()) return 1;

  cv::Mat frame;
  sensor_msgs::ImagePtr msg;

  ros::Rate loop_rate(25);
  while (ros::ok()) {
    cap >> frame;

    // Check if grabbed frame is actually full with some content
    if(!frame.empty()) {
      msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
      pub.publish(msg);
    }

    loop_rate.sleep();
  }

  return 0;
}
