#include "ros/ros.h"
#include <iostream>
#include <string>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/persistence.hpp>
#include "frameparam.cpp"
using namespace cv;
using namespace std;
bool getParamFromConfig(ros::NodeHandle nh,int *outCBW,int *outCBH,vector<string> &outImgpathlist,string &outdevicepath,string &outsavepath){
    if(!nh.getParam("images",outImgpathlist)){
        ROS_ERROR("No name images param");
        return false;
    }
    if(!nh.getParam("deivce_path",outdevicepath)){
        ROS_ERROR("No name deivce_path param");
        return false;
    }
    if(!nh.getParam("CHECKBOARD_WIDTH",*outCBW)){
        ROS_ERROR("No name CHECKBOARD_WIDTH param");
        return false;
    }
    if(!nh.getParam("CHECKBOARD_HEIGHT",*outCBH)){
        ROS_ERROR("No name CHECKBOARD_WIDTH param");
        return false;
    }
    if(!nh.getParam("OUTPUTSAVEPATH",outsavepath)){
        ROS_ERROR("No name OUTPUTSAVEPATH param");
        return false;
    }
  
    return true;
}
bool SaveFile(string filepath_name,vector<string> &inImgpathlist,Mat inframe,Mat incameraMatrix,Mat indistCoeffs,vector<Mat> &inRotation,vector<Mat> &inTranslation,double inrms)
{
    FileStorage fs;
    fs.open(filepath_name, FileStorage::WRITE|CV_STORAGE_FORMAT_YAML); // check file exist
    if (!fs.isOpened()){
        cerr << "Failed to open " << filepath_name << endl;
        fs.release();
    }
    fs<<"Overall"<<"{";
    if(inImgpathlist.size()==0)
        fs<<"Imagepath"<<inImgpathlist;
    fs<<"Image Width"<<inframe.size().width;
    fs<<"Image Height"<<inframe.size().height;
    fs<<"rms"<<inrms;
        fs<<"Intrinsic-Camera-Parameter"<<"{";
        fs<<"Focal x"<<incameraMatrix.at<double>(0,0);
        fs<<"Focal y"<<incameraMatrix.at<double>(1,1);
        fs<<"px cx "<<incameraMatrix.at<double>(0,2);
        fs<<"py cy"<<incameraMatrix.at<double>(1,2);
        fs<<"cameraMatrix"<<incameraMatrix;
        fs<<"}";
        fs<<"Distortion coefficients"<<"{";
        fs<< "k1" << indistCoeffs.at<double>(0,0);
        fs<< "k2" << indistCoeffs.at<double>(0,1);
        fs<< "p1" << indistCoeffs.at<double>(0,2);
        fs<< "p2" << indistCoeffs.at<double>(0,3);
        fs<< "k3" << indistCoeffs.at<double>(0,4);
        fs<<"distCoeffs"<<indistCoeffs;
        fs<<"}";
        fs<<"Extrinsic-Camera-Parameter"<<"{";// 9x9 matrix
        fs<<"Translation"<<inTranslation;
        fs<<"Rotation"<<inRotation;
        fs<<"}";
    fs<<"}";
    fs.release();
    return true;
}

int main(int argc , char** argv){
    ros::init(argc, argv, "ROS_CAM");
    ros::NodeHandle nh;
    bool Activate = false;
    if(!nh.getParam("isCali",Activate)){
        ROS_ERROR("Cannot retrieve Param");
    }
    if(!Activate)
        return 0;
    vector<vector<Point3f>> objpoints; // Store 3D points
    vector<vector<Point2f>> imgpoints; // Store 2D points
    Mat frame,cameraMatrix,distCoeffs;
    vector<Mat> Rotation,Translation;
    vector<string> Imgpathlist;
    int CBW,CBH;
    string devicepath,outputsavepath;
    if(getParamFromConfig(nh,&CBW,&CBH,Imgpathlist,devicepath,outputsavepath)){
        for(int i =0; i < Imgpathlist.size();i++){
            CBImagesParam img(Imgpathlist[i],CBW,CBH);
            img.RunCalibration();
            if(img.getFound()){
                objpoints.push_back(img.getobjp());
                imgpoints.push_back(img.getcorner_pts());
                frame = img.getImageFrame();
            }
            waitKey(0);
        }
    }
    else{
        VideoCapture cap(devicepath);
        for(int i =0;i<10;){
            cap.read(frame);
            if (frame.empty()) {
            ROS_ERROR("ERROR! blank frame grabbed");
            return 0;
            }
            int value = waitKeyEx(30);
            switch(value){
                case 27 : {ROS_INFO("Exit");return 0;break;}
                case 32 : { 
                            CBImagesParam img(frame,CBW,CBH);
                            img.RunCalibrationFrame();
                            if(img.getFound()){
                                 i++;
                                objpoints.push_back(img.getobjp());
                                imgpoints.push_back(img.getcorner_pts());
                                frame = img.getImageFrame();
                            }
                            break;
                } 
            }  
            cv::putText(frame, //target image
                        (to_string(i)+":10"), //text
                        cv::Point(10, frame.rows / 2), //top-left position
                        cv::FONT_HERSHEY_DUPLEX,
                        1.0,
                        CV_RGB(255, 255,0), //font color
                        2);
            imshow("live",frame);     
        }
    }
    double rms=cv::calibrateCamera(objpoints, imgpoints,Size(frame.rows,frame.cols), cameraMatrix, distCoeffs, Rotation, Translation);
    ROS_INFO("RMS Score : %f",rms);
    SaveFile(outputsavepath,Imgpathlist,frame,cameraMatrix,distCoeffs,Rotation,Translation,rms);
    Activate = false;
    nh.setParam("isCali",Activate);
    return 0;
}
