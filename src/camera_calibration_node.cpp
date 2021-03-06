#include"source.h"

using namespace std;
using namespace cv;

class calibration
{
public:
  calibration():
    it(nh),
    img_ptr(nullptr)
  {
    ros::NodeHandle private_nh("~");
    private_nh.getParam("params_path",params_path);

    images_sub       = it.subscribe("/image_raw", 20, &calibration::imageCallback,this);

    calibrate_srv    = nh.advertiseService("/calibrate",&calibration::calibrate,this);

    initialize(); //Set blobdetector params and subpix criteria

    startWindowThread();
  }
private:
  void imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
      lock_guard<mutex> lock(img_mutex);
      img_ptr = msg;
  }

  void initialize()
  {
    board_size = Size(4,11);
    params.maxArea=10e4;
    params.minArea=10;
    params.minDistBetweenBlobs=5;
    criteria = TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER,40,0.001);
  }

  void detectcenters(Mat image_gray, Mat& centers)
  {
    //Detect the center points of blobs
    findCirclesGrid(image_gray,
                    board_size,
                    centers,
                    CALIB_CB_ASYMMETRIC_GRID,
                    detector.create());

    //Get subpix for more precise result
    cornerSubPix(image_gray,
                 centers,
                 Size(5,5),
                 Size(-1,-1),
                 criteria);
  }

  bool calibrate(camera_calibration::calibrate::Request  &req,
                 camera_calibration::calibrate::Response &res)
  {
    // Return when there is no image accepted
    if(!img_ptr) return false;

    vector<Mat> total_centers;
    vector<vector<Point3f>> total_object;

    vector<Point3f> object_points;

    //Set blobs' position in real world with all blobs in the same plane
    //Consider blobs arrary in 5 rows with additional 4 blobs
    for(int i=0; i<5; i++)
    {
      for(int j=0; j<4; j++)
      {
        Point3f realPoint;
        realPoint.x = j*32;
        realPoint.y = i*32;
        realPoint.z = 0;
        object_points.push_back(realPoint);
        realPoint.x = j*32+16;
        realPoint.y = i*32+16;
        realPoint.z = 0;
        object_points.push_back(realPoint);
      }
    }
    for(int j=0; j<4; j++)
    {
      Point3f realPoint;
      realPoint.x = j*32;
      realPoint.y = 5*32;
      realPoint.z = 0;
      object_points.push_back(realPoint);
    }

    //Loading 20 pictures
    Size image_size;
    int picture_id;
    for (int i=0; i<20; i++)
    {
      Mat image_rgb  = cv_bridge::toCvShare(img_ptr, "bgr8") ->image,
          image_gray = cv_bridge::toCvShare(img_ptr, "mono8")->image;

      //Using image pyramid method to shrink images
      for (int j=0; j<3; j++)
      {
      pyrDown(image_rgb,image_rgb);
      pyrDown(image_gray,image_gray);
      }
      //Setting final image size for calibration param in
      image_size = image_gray.size();

      //Calculate centers and push them together
      Mat centers;
      detectcenters(image_gray,centers);
      total_centers.push_back(centers);
      total_object.push_back(object_points);
      picture_id = img_ptr->header.seq;

      //Draw center points in color images for visualization
      drawChessboardCorners(image_rgb,
                            board_size,
                            centers,
                            true);

      //Wait until next picture
      while(picture_id == img_ptr->header.seq)
      {
        imshow("chessboard",image_rgb);
        waitKey(1);
        ros::spinOnce();
      }

      //Announce how many pictures have got
      ROS_INFO("Picture Loading ....%i", i);
    }
    destroyAllWindows();

    //Calibrate
    Mat         cameraMatrix,
                distCoeffs;
    vector<Mat> rvecs,
                tvecs;

    calibrateCamera(total_object,
                    total_centers,
                    image_size,
                    cameraMatrix,
                    distCoeffs,
                    rvecs,
                    tvecs,
                    CV_CALIB_FIX_K3);

    //Write params as .yaml to 'params' folder in package
    FileStorage fs(params_path,FileStorage::WRITE);
    fs << "cameraMatrix" << cameraMatrix;
    fs << "distCoeffs"   << distCoeffs;
    fs << "rvecs"        << rvecs;
    fs << "tvecs"        << tvecs;

    fs.release();

    //Service call finished, return true
    ROS_INFO("Calibration Complete!");

    res.success = true;
    return res.success;
  }

private:
  ros::NodeHandle nh;
  ros::ServiceServer calibrate_srv;
  sensor_msgs::ImageConstPtr img_ptr;
  image_transport::ImageTransport it;
  image_transport::Subscriber images_sub;

  Size board_size;
  SimpleBlobDetector::Params params;
  SimpleBlobDetector detector;
  TermCriteria criteria;

  mutex img_mutex;
  string params_path;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "camera_calibration_node");

  calibration chessboard;

  ros::spin();

  return 0;
}
