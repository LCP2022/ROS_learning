#include <string>
#include <vector>
#include <iostream>

#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include "opencv2/imgproc.hpp"
class CBImagesParam
{
    private:
        int Chessboard_Width;
        int Chessboard_Height;
        cv::Mat imgframe;
        cv::Size Framesize;
        std::string Image_Path_Name;
        std::vector<cv::Point3f> objp; // Defining the world coordinates for 3D points
        std::vector<cv::Point2f> corner_pts; // Store  pixel coordinates of detected checker board corners 
        bool Found = false;
    public:
        CBImagesParam(std::string imgpath,int CBWidth,int CBHeight);
        CBImagesParam(cv::Mat frame ,int CBWidth,int CBHeight);
        void RunCalibration();
        void RunCalibrationFrame();
        void PrintImage_WH();
        void FindDrawChessboardCorners();
        bool getFound();
        std::vector<cv::Point3f> getobjp();
        std::vector<cv::Point2f> getcorner_pts();
        cv::Mat getImageFrame();

};

CBImagesParam::CBImagesParam(std::string filepathname,int CBWidth,int CBHeight)
{
    Chessboard_Height = CBHeight;
    Chessboard_Width = CBWidth;
    Image_Path_Name = filepathname;
}
CBImagesParam::CBImagesParam(cv::Mat frame ,int CBWidth,int CBHeight)
{
    Chessboard_Height = CBHeight;
    Chessboard_Width = CBWidth;
    imgframe = frame;
}
void CBImagesParam::RunCalibration(){
    
    imgframe = cv::imread(Image_Path_Name,cv::IMREAD_COLOR); 
    Framesize = imgframe.size();
    imshow("Orignal",imgframe);
    FindDrawChessboardCorners();
}
void CBImagesParam::RunCalibrationFrame(){
    Framesize = imgframe.size();
    //cv::imshow("Orignal",imgframe);
    FindDrawChessboardCorners();
}
void CBImagesParam::FindDrawChessboardCorners(){
    for(int i{0}; i<Chessboard_Height; i++)
        {
            for(int j{0}; j<Chessboard_Width; j++)
            objp.push_back(cv::Point3f(j,i,0));
        }
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
            cv::imshow("Drew",dummyframe);
        }
}
bool CBImagesParam::getFound(){
    return Found;
}
std::vector<cv::Point3f> CBImagesParam::getobjp(){
    return objp;
}
std::vector<cv::Point2f> CBImagesParam::getcorner_pts(){
    return corner_pts;
}
cv::Mat CBImagesParam::getImageFrame(){
    return imgframe;
}