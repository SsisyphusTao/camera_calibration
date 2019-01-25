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

    initialize();

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

  bool calibrate(camera_calibration::calibrate::Request  &req,
                 camera_calibration::calibrate::Response &res)
  {
    Mat image_rgb,
        image_gray;

    if(!img_ptr) return false;
    else
    {
      lock_guard<mutex> lock(img_mutex);
      image_rgb  = cv_bridge::toCvShare(img_ptr, "rgb8") ->image;
      image_gray = cv_bridge::toCvShare(img_ptr, "mono8")->image;
    }
    Mat centers;

    findCirclesGrid(image_gray,
                    board_size,
                    centers,
                    CALIB_CB_ASYMMETRIC_GRID,
                    detector.create(params));

    cornerSubPix(image_gray,
                 centers,
                 Size(5,5),
                 Size(-1,-1),
                 criteria);
    drawChessboardCorners(image_rgb,board_size,centers,false);
    imshow("chessboard",image_rgb);
    waitKey();
    destroyAllWindows();

    Mat         cameraMatrix,
                distCoeffs;
    vector<Mat> rvecs,
                tvecs;

    vector<Point3f> object_points;

    for(int i=0; i<board_size.height;i++)
    {
      for(int j=0; j<board_size.width;j++)
      {
        Point3f realPoint;
        realPoint.x = i*10;
        realPoint.y = j*10;
        realPoint.z = 0;
        object_points.push_back(realPoint);
      }
    }
    vector<Mat> total_centers;
    vector<vector<Point3f>> total_object;

    for (int i = 0 ; i<20;i++)
    {
      total_centers.push_back(centers);
      total_object.push_back(object_points);
    }

    calibrateCamera(total_object,
                    total_centers,
                    image_gray.size(),
                    cameraMatrix,
                    distCoeffs,
                    rvecs,
                    tvecs,
                    CV_CALIB_FIX_K3);

    FileStorage fs(params_path,FileStorage::WRITE);
    fs << "cameraMatrix" << cameraMatrix;
    fs << "distCoeffs"   << distCoeffs;
    fs << "rvecs"        << rvecs;
    fs << "tvecs"        << tvecs;

    fs.release();
    res.success = true;
    return res.success;
  }

private:
  ros::NodeHandle nh;
  ros::Timer image_loop_timer;
  ros::ServiceServer calibrate_srv;
  sensor_msgs::ImageConstPtr img_ptr;
  image_transport::ImageTransport it;
  image_transport::Subscriber images_sub;
  double image_loop_duration;
  mutex img_mutex;
  string params_path;

  Size board_size;
  SimpleBlobDetector::Params params;
  SimpleBlobDetector detector;

  TermCriteria criteria;


};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "camera_calibration_node");

  calibration chessboard;

  ros::spin();

  return 0;
}
