#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "camera_node");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("camera/image/grayscale", 1);

  // Convert the passed as command line parameter index for the video device to an integer
  std::istringstream video_sourceCmd("0");
  int video_source;
  // Check if it is indeed a number
  if(!(video_sourceCmd >> video_source)) return 1;

  cv::VideoCapture cap(video_source);
  // Check if video device can be opened with the given index
  if(!cap.isOpened()) return 1;
  cv::Mat frame, frame_gray;
  sensor_msgs::ImagePtr msg;

  ros::Rate loop_rate(50);
  while (nh.ok()) {
    cap >> frame;

    if(!frame.empty()) {
      cv::cvtColor( frame, frame_gray, CV_BGR2GRAY );
      msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", frame_gray).toImageMsg();
      pub.publish(msg);
      cv::waitKey(1);
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
}
