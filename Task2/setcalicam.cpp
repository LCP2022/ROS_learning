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
    Mat cameraMatrix,distCoeffs,R,T;
};
cv::Scalar BLUE = cv::Scalar(255,0,0);
cv::Scalar GREEN = cv::Scalar(0,255,0);
cv::Scalar RED = cv::Scalar(0,0,255);
cv::Scalar YELLOW = cv::Scalar(0,255,255);


bool LoadFile(string filepath_name ,FileStorage &outfs){
    outfs.open(filepath_name, FileStorage::READ|CV_STORAGE_FORMAT_YAML);
    if (!outfs.isOpened()){
        cerr << "Failed to open " << filepath_name << endl;
        return false;
    }
    return true;
}
CameraCablparam LoadCalibrateSetting(FileStorage fs){
    CameraCablparam ccparam ;
    FileNode root = fs["Overall"];
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

void pose_estimation(Mat in_image,CameraCablparam cali)
{   float boxwidth = 1;
    std::vector<cv::Point3f> cubePoints = {
        {0, 0, 0}, {0, boxwidth, 0}, {boxwidth, boxwidth, 0}, {boxwidth, 0, 0},
        {0, 0, -boxwidth}, {0, boxwidth, -boxwidth}, {boxwidth, boxwidth, -boxwidth}, {boxwidth, 0, -boxwidth}
    };
    std::vector<cv::Point2f> imgptsss;
    cv::projectPoints(cubePoints,cali.R,cali.T,cali.cameraMatrix,cali.distCoeffs,imgptsss);
    
   // Draw cube edges
    cv::polylines(in_image, std::vector<std::vector<cv::Point>>{{imgptsss[0], imgptsss[1], imgptsss[2], imgptsss[3]}}, true,GREEN, 2); // front face
    cv::polylines(in_image, std::vector<std::vector<cv::Point>>{{imgptsss[4], imgptsss[5], imgptsss[6], imgptsss[7]}}, true, BLUE, 2); // back face
    for (int i = 0; i < 4; ++i) {
        cv::line(in_image, imgptsss[i], imgptsss[i + 4], RED, 2); // connecting lines
    }
  
   // cv::line(in_image,YELLOW,4);
    cv::imshow("pose estimation",in_image);
}

int main(int argc , char** argv){
    FileStorage fs;
    bool status = LoadFile("/home/user/Desktop/CaliCam/src/piccali/config/Output.yaml",fs);
    CameraCablparam listparam;
    listparam = LoadCalibrateSetting(fs);
    Mat frame = imread(listparam.Imagepath);
    fs.release();
    return 0;
}

