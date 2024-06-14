#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <string>
#include <vector>
using namespace cv;
using namespace std;




bool LoadFile(string filepath_name, vector<string> &listvar){
    FileStorage fs;
    fs.open(filepath_name, FileStorage::READ);
    if (!fs.isOpened()){
        cerr << "Failed to open " << filepath_name << endl;
        return false;
    }
    // look for <image></images> with multi string
    FileNode fn = fs["images"]; 
    if (fn.type() != FileNode::SEQ){
        cerr << "strings is not a sequence! FAIL" << endl;
        return false;
    }
    FileNodeIterator it = fn.begin(), it_end = fn.end(); // Go through the node
    for (; it != it_end; ++it){
        listvar.push_back((string)*it);
    }
    cout <<"Load to Image path : "<< listvar.size() << endl;
    fs.release();
    return true;
}





int main(int argc , char** argv){
    vector<string> Imgpathlist;
    LoadFile("/home/user/Desktop/CaliCam/src/piccali/config/InputSetting.xml",Imgpathlist);
    for(int i =0; i < Imgpathlist.size();i++){
        Mat imgframe = imread(Imgpathlist[i],IMREAD_COLOR);
        imshow("Live",imgframe);
        Size boardsize = imgframe.size();
        cout<<"Width :"<<boardsize.width<< " Height :" << boardsize.height << endl;
        vector<Mat> rvecs, tvecs;
        vector<float> reprojErrs;
        Mat cameraMatrix = Mat::eye(3, 3, CV_64F);
        Mat distCoeffs = Mat::zeros(8, 1, CV_64F);
        vector<vector<Point3f> > objectPoints(1);




        vector<Point3f> pointBuf;
        bool found = findChessboardCorners(imgframe, boardsize, pointBuf, CALIB_CB_FAST_CHECK );
        cout<<found<<endl;
        waitKey(0); 
    }   
    return 0;
}