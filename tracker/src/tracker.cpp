#include "ros/ros.h"
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "dsst_c.h"

int run_tracker(int argc, char **argv)
{
  // Set up the application state for the MATLAB Runtime instance created in the application.
  if (!mclInitializeApplication(NULL,0))
    {
      std::cerr << "could not initialize the application properly"
                << std::endl;
          return -1;
    }

  // Load the required MATLAB code into the MATLAB Runtime.
  if( !addmatrixInitialize() )
    {
      std::cerr << "could not initialize the library properly"
                << std::endl;
             return -1;
    }

  try
    {
      // Create input data
      double data[] = {1,2,3,4,5,6,7,8,9};
      mwArray in1(3, 3, mxDOUBLE_CLASS, mxREAL);
      mwArray in2(3, 3, mxDOUBLE_CLASS, mxREAL);
      in1.SetData(data, 9);
      in2.SetData(data, 9);

      // Create output array
      mwArray out;

      // Call the library function
      addmatrix(1, out, in1, in2);

      std::cout << "The value of added matrix is:" << std::endl;
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

  // Release the resources used by the generated MATLAB code
  addmatrixTerminate();

  // Release all state and resources used by the MATLAB Runtime for the application
  mclTerminateApplication();
  return 0;
}

int main()
{
  // Initialize the MATLAB Runtime
  mclmcrInitialize();

  // Create a new thread and run the MATLAB generated code in it.
  return mclRunMain((mclMainFcnType)run_tracker,0,NULL);
}

void chatterCallback(const sensor_msgs::ImageConstPtr& msg)
{
  ROS_DEBUG("I heard: []");
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "tracker");

  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("/camera/image_raw", 300, chatterCallback);

  ros::spin();

  return 0;
}
