#include "ros/ros.h"
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "libtest.h"

void chatterCallback(const sensor_msgs::ImageConstPtr& msg)
{
  ROS_DEBUG("I heard: []");
}

int run_tracker(int argc, char **argv)
{
  ros::init(argc, argv, "tracker");

  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("/camera/image_raw", 300, chatterCallback);

  // Set up the application state for the MATLAB Runtime instance created in the application.
  if (!mclInitializeApplication(NULL, 0))
  {
    std::cerr << "could not initialize the application properly" << std::endl;
    return -1;
  }

  // Load the required MATLAB code into the MATLAB Runtime.
  if (!libtestInitialize())
  {
    std::cerr << "could not initialize the library properly" << std::endl;
    return -1;
  }

  ros::Rate loop_rate(25);

  while (ros::ok())
  {
    try
    {
      // Create input data
      double data[] = {3, 4, 5, 6, 7, 8, 9};
      mwArray in(1, 1, mxDOUBLE_CLASS, mxREAL);
      in.SetData(data, 1);

      // Create output array
      mwArray out;

      // Call the library function
      libtest(1, out, in);

      std::cout << "The value is:" << std::endl;
      std::cout << out << std::endl;
    }

    // Catch the MATLAB generated mwException
    catch (const mwException& e)
    {
      std::cerr << e.what() << std::endl;
      return -2;
    }

    // Catch any other exceptions that may be thrown
    catch (...)
    {
      std::cerr << "Unexpected error thrown" << std::endl;
      return -3;
    }

    ros::spinOnce();

    loop_rate.sleep();
  }

  // Release the resources used by the generated MATLAB code
  libtestTerminate();

  // Release all state and resources used by the MATLAB Runtime for the application
  mclTerminateApplication();

  return 0;
}

int main(int argc, char **argv)
{
  // Initialize the MATLAB Runtime
  mclmcrInitialize();

  // Create a new thread and run the MATLAB generated code in it.
  return mclRunMain((mclMainFcnType)run_tracker, 0, NULL);
}
