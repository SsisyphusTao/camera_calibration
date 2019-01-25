#include"source.h"

using namespace std;
using namespace cv;
int main(int argc, char **argv)
{
  ros::init(argc, argv, "images_pub");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  //Getting pictures' path in ROS package
  string images_path;
  private_nh.getParam("images_path",images_path);

  ros::Publisher images_pub = nh.advertise<sensor_msgs::Image>("/image_raw", 20);

  //Sending pictures in 2Hz
  ros::Rate loop_rate(2);

  int sequence = 0;

  while (ros::ok())
  {
    //Using sequence to locate pictures ID
    char image_id[5];
    string image_name;
    sprintf(image_id,"%i",sequence%20);
    image_name = image_id;
    Mat photo = imread(images_path+image_name+".jpg");

    cv_bridge::CvImage image;
    std_msgs::Header header;

    header.frame_id = image_name;
    header.stamp    = ros::Time::now();
    header.seq      = sequence;
    sequence++;

    //Sending image as BGR format
    image.header = header;
    image.encoding = sensor_msgs::image_encodings::BGR8;
    image.image = photo;

    images_pub.publish(image.toImageMsg());

    loop_rate.sleep();
  }

  return 0;
}
