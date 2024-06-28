#include "ros/ros.h"
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include "opencv2/imgproc.hpp"
#include <opencv2/core/persistence.hpp>
#include <string>
#include <vector>
using namespace cv;
using namespace std;
struct CameraCablparam{
    string Imagepath;
    int width,height;
    double fx,fy,cx,cy,k1,k2,p1,p2,k3,rms;
    Mat cameraMatrix,distCoeffs;
    vector<Mat> R,T;
    vector<vector<Point2f>> imgpoints; // Store 2D points
};
cv::Scalar BLUE = cv::Scalar(255,0,0);
cv::Scalar GREEN = cv::Scalar(0,255,0);
cv::Scalar RED = cv::Scalar(0,0,255);
cv::Scalar YELLOW = cv::Scalar(0,255,255);
CameraCablparam LoadCalibrateSetting(ros::NodeHandle nh){
    CameraCablparam ccparam ;
    string inputconfigfile;
    if(!nh.getParam("OUTPUTSAVEPATH",inputconfigfile)){
        ROS_ERROR("No name OUTPUTSAVEPATH param");
        return ccparam;
    }
    // nh.getParam("Imagepath",ccparam.Imagepath)
    // nh.getParam("Image Width",ccparam.width)
    // nh.getParam("Image Height",ccparam.height)
    // nh.getParam("rms",ccparam.rms)
    // nh.getParam("OUTPUTSAVEPATH",ccparam.fx)
    // nh.getParam("OUTPUTSAVEPATH",ccparam.fy)
    // nh.getParam("OUTPUTSAVEPATH",ccparam.cx)
    // nh.getParam("OUTPUTSAVEPATH",ccparam.cy)
    // nh.getParam("OUTPUTSAVEPATH",ccparam.cameraMatrix)
    // nh.getParam("OUTPUTSAVEPATH",ccparam.k1)
    // nh.getParam("OUTPUTSAVEPATH",ccparam.k2)
    // nh.getParam("OUTPUTSAVEPATH",ccparam.p1)
    // nh.getParam("OUTPUTSAVEPATH",ccparam.p2)
    // nh.getParam("OUTPUTSAVEPATH",ccparam.k3)

    FileStorage fs;
    FileNode root = fs[inputconfigfile];
    root["Imagepath"] >> ccparam.Imagepath ;
    root["Image Width"] >> ccparam.width ;
    root["Image Height"] >> ccparam.height ;
    root["rms"] >> ccparam.rms;

    root["Intrinsic-Camera-Parameter"]["Focal x"]>>ccparam.fx;
    root["Intrinsic-Camera-Parameter"]["Focal y"]>>ccparam.fy;
    root["Intrinsic-Camera-Parameter"]["px cx"]>>ccparam.cx;
    root["Intrinsic-Camera-Parameter"]["py cy"]>>ccparam.cy;
    root["Intrinsic-Camera-Parameter"]["cameraMatrix"]>>ccparam.cameraMatrix;

    root["Distortion coefficients"]["k1"]>>ccparam.k1;
    root["Distortion coefficients"]["k2"]>>ccparam.k2;
    root["Distortion coefficients"]["p1"]>>ccparam.p1;
    root["Distortion coefficients"]["p2"]>>ccparam.p2;
    root["Distortion coefficients"]["k3"]>>ccparam.k3;
    root["Distortion coefficients"]["distCoeffs"]>>ccparam.distCoeffs;
    root["Extrinsic-Camera-Parameter"]["Translation"]>>ccparam.T;
    root["Extrinsic-Camera-Parameter"]["Rotation"]>>ccparam.R;
    return ccparam;
}
void pose_estimation(Mat in_image,CameraCablparam cali,vector<Point2f> incorner_pts,  vector<Point3f> inobjp)
{   
    float boxwidth = 1;
    std::vector<cv::Point3f> cubePoints = {
        {0, 0, 0}, {0, boxwidth, 0}, {boxwidth, boxwidth, 0}, {boxwidth, 0, 0},
        {0, 0, -boxwidth}, {0, boxwidth, -boxwidth}, {boxwidth, boxwidth, -boxwidth}, {boxwidth, 0, -boxwidth}
    };
    
    std::vector<cv::Point2f> imgptsss;
    Mat Rot = cv::Mat::zeros(1, 3, CV_32F);
    Mat Tran = cv::Mat::zeros(1, 3, CV_32F);
    bool ok =solvePnP(inobjp,incorner_pts,cali.cameraMatrix,cali.distCoeffs,Rot,Tran,false,SOLVEPNP_ITERATIVE);
//     cv::projectPoints(cubePoints,Rot,Tran,cali.cameraMatrix,cali.distCoeffs,imgptsss);
//     Mat dummyimage;
//     in_image.copyTo(dummyimage);
//    // Draw cube edges
//     cv::polylines(dummyimage, vector<vector<Point>>{{imgptsss[0], imgptsss[1], imgptsss[2], imgptsss[3]}}, true,GREEN, 2); // front face
//     cv::polylines(dummyimage, vector<vector<Point>>{{imgptsss[4], imgptsss[5], imgptsss[6], imgptsss[7]}}, true, BLUE, 2); // back face
//     for (int i = 0; i < 4; ++i) {
//         cv::line(dummyimage, imgptsss[i], imgptsss[i + 4], RED, 2); // connecting lines
//     }
  
//    // cv::line(in_image,YELLOW,4);
//     cv::imshow("pose estimation",dummyimage);
}
void undistoredtheimage(Mat in_image,CameraCablparam cali){
    Mat outimage;
    cv::undistort(in_image, outimage, cali.cameraMatrix, cali.distCoeffs);
    imshow("undistored",outimage);
}

int main(int argc , char** argv){
    ros::init(argc, argv, "ROS_CAM_Pose_Estimation");
    ros::NodeHandle nh;
    bool Calidone = true;
    int Chessboard_Width,Chessboard_Height;
    string devicepath;
    while(Calidone){
        if(!nh.getParam("isCali", Calidone)){ 
            ROS_ERROR("No name param");
            return 0;
        }
    }
    CameraCablparam listparam;
    listparam = LoadCalibrateSetting(nh);
    nh.getParam("deivce_path",devicepath);
    nh.getParam("CHECKBOARD_WIDTH",Chessboard_Width);
    nh.getParam("CHECKBOARD_HEIGHT",Chessboard_Height);
    vector<Point3f> objp; // Defining the world coordinates for 3D points
    for(int i{0}; i<Chessboard_Height; i++){
        for(int j{0}; j<Chessboard_Width; j++)
        objp.push_back(cv::Point3f(j,i,0));
    }
    VideoCapture cap(devicepath);
    if (!cap.isOpened()) {
    ROS_ERROR("ERROR! Unable to open camera");
    return -1;
    }
    Mat Frame,gray;
    while(1){
        cap.read(Frame);
        if (Frame.empty()) {
        ROS_ERROR("ERROR! blank frame grabbed");
        break;
        }
        cvtColor(Frame,gray,cv::COLOR_BGR2GRAY);
        std::vector<cv::Point2f> corner_pts; // Store  pixel coordinates of detected checker board corners 
        bool Found = cv::findChessboardCorners(gray, cv::Size(Chessboard_Width,Chessboard_Height), corner_pts, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
        if(Found){
            
            cv::TermCriteria criteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.001);
            // refining pixel coordinates for given 2d points.
            cv::cornerSubPix(gray,corner_pts,cv::Size(11,11), cv::Size(-1,-1),criteria);
//            pose_estimation(Frame,listparam,corner_pts,objp);
            undistoredtheimage(Frame,listparam);

        }
        imshow("LiveView",Frame);
        int value = waitKeyEx(30);
        switch(value){
            case 27 : return 0;break;
        }
    }

    return 0;
}