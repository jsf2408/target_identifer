#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/ml/ml.hpp>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <sstream>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/NavSatFix.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"

using namespace cv;
using namespace std;

static const std::string OPENCV_WINDOW = "Image window";
// global variables ///////////////////////////////////////////////////////////////////////////////
const int MIN_CONTOUR_AREA = 1000;
const int RESIZED_IMAGE_WIDTH = 80;
const int RESIZED_IMAGE_HEIGHT = 80;
const int RESIZE_WIDTH = 800;
const int RESIZE_HEIGHT = 600;
int target_size;

int lat;
int lon;

///////////////////////////////////////////////////////////////////////////////////////////////////
class ContourWithData {
public:
        // member variables ///////////////////////////////////////////////////////////////////////////
        vector<Point> ptContour;           // contour
        cv::Rect boundingRect;                      // bounding rect for contour
        float fltArea;                              // area of contour
        int height;
        int width;
        ///////////////////////////////////////////////////////////////////////////////////////////////
        bool checkIfContourIsValid() {                              // obviously in a production grade program
                width = boundingRect.width;
                height = boundingRect.height;
                if ((width / height) >= 0.995 && (width / height) <= 1.005) {
                        if ((fltArea > target_size*0.8) && (fltArea < target_size*1.2)) {
                                return true;
                        } else {
                                return false;
                        }
                } else {
                        return false;                                            // identifying if a contour is valid !!
                }
        }
        ///////////////////////////////////////////////////////////////////////////////////////////////
        static bool sortByBoundingRectXPosition(const ContourWithData& cwdLeft, const ContourWithData& cwdRight) {      // this function allows us to sort
                return(cwdLeft.boundingRect.height*cwdLeft.boundingRect.width < cwdRight.boundingRect.height*cwdRight.boundingRect.width);                                                   // the contours from left to right
        }

};


void targetCb(const std_msgs::Float32::ConstPtr& msg)
{
	target_size = msg->data;
}


void gpsCb(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
	lat = msg->latitude;
	lon = msg->longitude;
}


class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Subscriber target_size_sub;
  ros::Subscriber gps_sub;
  ros::Publisher gps_pub;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/cv_camera/image_raw", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/identifier/output_image", 1);
    target_size_sub = nh_.subscribe("target_size", 1 , targetCb);
    gps_sub = nh_.subscribe("mavros/global_position/global", 1 , gpsCb);
    gps_pub = nh_.advertise<std_msgs::Float32MultiArray>("identifier/gps",1);
    //cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    //cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    cv::Mat input = cv_ptr->image;
cv::resize(input, input, Size(RESIZE_WIDTH, RESIZE_HEIGHT));
    cv::cvtColor(input, input, CV_BGR2GRAY,1);         // convert to grayscale																 // blur

    cv::GaussianBlur(input, input, Size(5,5), 1.8);

    cv::adaptiveThreshold(input, input, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 35, -2);
    cv::Mat output = input.clone();

    cv::Canny(input, input, 50, 100);
    // Update GUI Window
    //cv::imshow(OPENCV_WINDOW, input);
    cv::waitKey(3);

    vector<vector<Point> > ptContours; // declare a vector for the contours
    vector<Vec4i> v4iHierarchy; // declare a vector for the hierarchy (we won't use this in this program but this may be helpful for reference)

    vector<ContourWithData> allContoursWithData;           // declare empty vectors,
    vector<ContourWithData> validContoursWithData;         // we will fill these shortly

    findContours(output, // input image, make sure to use a copy since the function will modify this image in the course of finding contours
            ptContours, // output contours
            v4iHierarchy, // output hierarchy
            RETR_EXTERNAL, // retrieve the outermost contours only
            CHAIN_APPROX_SIMPLE); // compress horizontal, vertical, and diagonal segments and leave only their end points

    for (int i = 0; i < ptContours.size(); i++) { // for each contour
            ContourWithData contourWithData; // instantiate a contour with data object
            contourWithData.ptContour = ptContours[i]; // assign contour to contour with data
            contourWithData.boundingRect = boundingRect(contourWithData.ptContour); // get the bounding rect
            contourWithData.fltArea = contourArea(contourWithData.ptContour); // calculate the contour area
            //contourWithData.target_size = target_size;
            allContoursWithData.push_back(contourWithData); // add contour with data object to list of all contours with data
    }

    for (int i = 0; i < allContoursWithData.size(); i++) { // for all contours
            if (allContoursWithData[i].checkIfContourIsValid()) { // check if valid
                    validContoursWithData.push_back(allContoursWithData[i]); // if so, append to valid contour list
            }
    }
    // sort contours from left to right
    sort(validContoursWithData.begin(), validContoursWithData.end(), ContourWithData::sortByBoundingRectXPosition);

    int size = validContoursWithData.size();

    if (size != -1){
        for (int i = 0; i < size; i++) { // for each contour

            cv::Mat matROI = output(validContoursWithData[i].boundingRect); // get ROI image of bounding rect

            cv::Mat matROIResized;
            cv::Mat fin;
            cv::resize(matROI, matROIResized, Size(RESIZED_IMAGE_WIDTH, RESIZED_IMAGE_HEIGHT)); // resize image, this will be more consistent for recognition and storage
            cv::rectangle(matROIResized, Point(0,0), Point(79,79), Scalar(255), 10);
            cv::floodFill(matROIResized, Point(0,0), Scalar(0));
            int morph_size = 1;
            cv::Mat element = getStructuringElement( MORPH_RECT, Size( 2*morph_size, 2*morph_size), Point( morph_size, morph_size ) );
            //cv::morphologyEx( matROIResized, matROIResized, MORPH_OPEN, element);
            //cv::morphologyEx( matROIResized, matROIResized, MORPH_CLOSE, element);

            //cv::Mat roi(100,100,CV_8UC1,Scalar(0));
            //cv::rectangle(roi, Point(0,0), Point(99,99), Scalar(0), 5);
            //matROIResized.copyTo(roi(Rect(10,10,fin.cols,fin.rows)));

            cv::cvtColor(matROIResized,matROIResized,CV_GRAY2BGR);
            cv_ptr->image = matROIResized;// Output modified video stream
            image_pub_.publish(cv_ptr->toImageMsg());

	    std_msgs::Float32MultiArray latLong;
	    latLong.data.clear();
	    latLong.data.push_back(lat);
	    latLong.data.push_back(lon);
	    gps_pub.publish(latLong);
            ros::spinOnce();
        }
    }

  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "identifier");
  ImageConverter ic;
  ros::spin();
  return 0;
}
