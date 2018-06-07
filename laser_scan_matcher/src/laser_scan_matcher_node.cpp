#include <laser_scan_matcher/laser_scan_matcher.h>
#include <gsl/gsl_vector_double.h>
using  namespace std;

void callback(sensor_msgs::LaserScan scan){

}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "LaserScanMatcher");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  scan_tools::LaserScanMatcher laser_scan_matcher(nh, nh_private);
  ros::Rate loop_rate(10);
  ros::spin();
/*  while(1){
    std::cout<<laser_scan_matcher.output_.x[0];
    if(laser_scan_matcher.output_.x[0]!=0)
      break;
    ROS_INFO("as");
    loop_rate.sleep();
    ros::spinOnce();
    ROS_INFO("as_1");
  }

  std::cout<<laser_scan_matcher.output_.x[0]<<"  y:"<<laser_scan_matcher.output_.x[1]<<"  "<<laser_scan_matcher.output_.x[2]<<std::endl;



  ros::Subscriber sub=nh.subscribe<sensor_msgs::LaserScan>("scan",100,callback);*/
  return 0;
}
