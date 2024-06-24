#include <string>
#include <vector>
#include <iostream>

#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include "opencv2/imgproc.hpp"


// For individual image
class Calibration
{
    private:
        int Chessboard_Width;
        int Chessboard_Height;
        cv::Mat imgframe;
        cv::Size Framesize;
        std::string Image_Path_Name;
        std::vector<std::vector<cv::Point3f>> objpoints; // Store 3D points
        std::vector<std::vector<cv::Point2f>> imgpoints; // Store 2D points
        std::vector<cv::Point3f> objp; // Defining the world coordinates for 3D points
        std::vector<cv::Point2f> corner_pts; // Store  pixel coordinates of detected checker board corners 
        cv::Mat cameraMatrix,distCoeffs,Rotation,Translation;
        double rms ;//overall RMS re-projection error
    public:
        cv::Scalar BLUE = cv::Scalar(255,0,0);
        cv::Scalar GREEN = cv::Scalar(0,255,0);
        cv::Scalar RED = cv::Scalar(0,0,255);
        cv::Scalar YELLOW = cv::Scalar(0,255,255);
        Calibration(std::string imgpath,int CBWidth,int CBHeight);
        Calibration(cv::Mat frame ,int CBWidth,int CBHeight);
        void RunCalibration();
        void RunCalibrationFrame();
        void FindDrawChessboardCorners(cv::Mat imgframe);
        cv::Mat RealRotation(cv::Mat R);
        void PrintIntrinsicParam();
        void PrintExtrinsicParam();
        void PrintImage_WH();
        void pose_estimation(cv::Mat in_image);
        cv::Mat getCameraMatrix();
        cv::Mat getDistCoeffs();
        cv::Mat getRotation();
        cv::Mat getTranslation();
        cv::Size getFramesize();
        cv::Mat UndistoredImage(cv::Mat in_image);
};
Calibration::Calibration(std::string filepathname,int CBWidth,int CBHeight)
{
    Chessboard_Height = CBHeight;
    Chessboard_Width = CBWidth;
    Image_Path_Name = filepathname;
}
Calibration::Calibration(cv::Mat frame ,int CBWidth,int CBHeight)
{
    Chessboard_Height = CBHeight;
    Chessboard_Width = CBWidth;
    imgframe = frame;
}
void Calibration::RunCalibration(){
    
   imgframe = cv::imread(Image_Path_Name,cv::IMREAD_COLOR); 
    Framesize = imgframe.size();
    imshow("Orignal",imgframe);
    FindDrawChessboardCorners(imgframe);
}
void Calibration::RunCalibrationFrame(){
    Framesize = imgframe.size();
    //cv::imshow("Orignal",imgframe);
    FindDrawChessboardCorners(imgframe);
}
void Calibration::FindDrawChessboardCorners(cv::Mat imgframe){
    for(int i{0}; i<Chessboard_Height; i++)
        {
            for(int j{0}; j<Chessboard_Width; j++)
            objp.push_back(cv::Point3f(j,i,0));
        }
    bool Found = false;
    cv::Mat gray;
    cv::cvtColor(imgframe,gray,cv::COLOR_BGR2GRAY);
    Found = cv::findChessboardCorners(gray, cv::Size(Chessboard_Width,Chessboard_Height), corner_pts, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
    if(Found)
        {
            cv::TermCriteria criteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.001);
            // refining pixel coordinates for given 2d points.
            cv::cornerSubPix(gray,corner_pts,cv::Size(11,11), cv::Size(-1,-1),criteria);
            // Displaying the detected corner points on the checker board
            cv::Mat dummyframe;
            imgframe.copyTo(dummyframe);
            cv::drawChessboardCorners(dummyframe, cv::Size(Chessboard_Width, Chessboard_Height), corner_pts, Found);
            objpoints.push_back(objp);
            imgpoints.push_back(corner_pts);
            cv::imshow("Found ChessBoard Corner Image",dummyframe);
            
            rms=cv::calibrateCamera(objpoints, imgpoints, cv::Size(gray.rows,gray.cols), cameraMatrix, distCoeffs, Rotation, Translation);
            PrintIntrinsicParam();
            PrintExtrinsicParam();

            cv::Mat undistimg = UndistoredImage(imgframe);
            //cv::imshow("undistoredimage",undistimg);
            pose_estimation(imgframe);
        }
}
cv::Mat Calibration::RealRotation(cv::Mat R){
    cv::Mat realR;
    Rodrigues(R,realR);
    return realR;
}
void Calibration::PrintIntrinsicParam(){
    std::cout<<"intrinsic camera Parameter"<<std::endl; // 9x9 matrix
    std::cout<<"Focal x : "<<cameraMatrix.at<double>(0,0)<<std::endl;
    std::cout<<"Focal y : "<<cameraMatrix.at<double>(1,1)<<std::endl;
    std::cout<<"px|cx : "<< cameraMatrix.at<double>(0,2)<<std::endl;
    std::cout<<"py|cy : "<< cameraMatrix.at<double>(1,2)<<std::endl;
    std::cout<<"distortion"<<std::endl; // 9x9 matrix
    std::cout<< "k1 : " << distCoeffs.at<double>(0,0)<<std::endl;
    std::cout<< "k2 : " << distCoeffs.at<double>(0,1)<<std::endl;
    std:: cout<< "p1 : " << distCoeffs.at<double>(0,2)<<std::endl;
    std::cout<< "p2 : " << distCoeffs.at<double>(0,3)<<std::endl;
    std::cout<< "k3 : " << distCoeffs.at<double>(0,4)<<std::endl;
}
void Calibration::PrintExtrinsicParam(){
    std::cout<<"extrinsic camera Parameter"<<std::endl; // 9x9 matrix
    std::cout<<"Translation : "<<Translation<<std::endl;
    std::cout<<"[3x3] Rotation : "<<RealRotation(Rotation)<<std::endl; //3x3
}
void Calibration::PrintImage_WH(){
    std::cout<<"Width :"    <<  Framesize.width;
    std::cout<<" Height :"  <<  Framesize.height << std::endl;
}
cv::Mat Calibration::UndistoredImage(cv::Mat in_image){
    // only work on fisheye images
    // some part of the image will be missing, this is ok, as it does not always get the full images correctly 
    cv::Mat out_image;
    cv::Mat map1,map2;
    cv::undistort( in_image, out_image, cameraMatrix, distCoeffs, cameraMatrix );
    std::cout<<rms<<std::endl;
    return out_image; 
}
void Calibration::pose_estimation(cv::Mat in_image)
{   float boxwidth = 1;
    std::vector<cv::Point3f> cubePoints = {
        {0, 0, 0}, {0, boxwidth, 0}, {boxwidth, boxwidth, 0}, {boxwidth, 0, 0},
        {0, 0, -boxwidth}, {0, boxwidth, -boxwidth}, {boxwidth, boxwidth, -boxwidth}, {boxwidth, 0, -boxwidth}
    };
    std::vector<cv::Point2f> imgptsss;
    cv::projectPoints(cubePoints,Rotation,Translation,cameraMatrix,distCoeffs,imgptsss);
    
   // Draw cube edges
    cv::polylines(in_image, std::vector<std::vector<cv::Point>>{{imgptsss[0], imgptsss[1], imgptsss[2], imgptsss[3]}}, true,GREEN, 2); // front face
    cv::polylines(in_image, std::vector<std::vector<cv::Point>>{{imgptsss[4], imgptsss[5], imgptsss[6], imgptsss[7]}}, true, BLUE, 2); // back face
    for (int i = 0; i < 4; ++i) {
        cv::line(in_image, imgptsss[i], imgptsss[i + 4], RED, 2); // connecting lines
    }
  
   // cv::line(in_image,YELLOW,4);
    cv::imshow("pose estimation",in_image);
}

cv::Mat Calibration::getCameraMatrix(){
    return cameraMatrix;
}
cv::Mat Calibration::getDistCoeffs(){
    return distCoeffs;
}
cv::Mat Calibration::getRotation(){
    return Rotation;
}
cv::Mat Calibration::getTranslation(){
    return Translation;
}
cv::Size Calibration::getFramesize(){
    return Framesize;
}

