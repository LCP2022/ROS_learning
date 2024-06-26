#include <iostream>
#include <string>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/persistence.hpp>
#include "frameparam.cpp"
using namespace cv;
using namespace std;
bool getconifgseq(FileStorage fs,string name,vector<string> &listvar){
    // look for <image></images> with multi string
    FileNode fn = fs[name]; 
    if (fn.type() != FileNode::SEQ){
        cerr << "strings is not a sequence! FAIL" << endl;
        return false;
    }
    FileNodeIterator it = fn.begin(), it_end = fn.end(); // Go through the node
    for (; it != it_end; ++it){
        listvar.push_back((string)*it);
    }
}
bool LoadConfigFile(string filepath_name ,FileStorage &outfs){
    outfs.open(filepath_name, FileStorage::READ);
    if (!outfs.isOpened()){
        cerr << "Failed to open " << filepath_name << endl;
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
    vector<vector<Point3f>> objpoints; // Store 3D points
    vector<vector<Point2f>> imgpoints; // Store 2D points
    Mat frame,cameraMatrix,distCoeffs;
    vector<Mat> Rotation,Translation;
    vector<string> Imgpathlist;
    FileStorage fs;
    bool status = LoadConfigFile("/home/user/Desktop/CaliCam/src/piccali/config/InputSetting.xml",fs);
    //status=0;
    if(status){
        getconifgseq(fs,"images",Imgpathlist);
        for(int i =0; i < Imgpathlist.size();i++){
            CBImagesParam img(Imgpathlist[i],fs["CHECKBOARD_WIDTH"],fs["CHECKBOARD_HEIGHT"]);
            img.RunCalibration();
            if(img.getFound()){
                objpoints.push_back(img.getobjp());
                imgpoints.push_back(img.getcorner_pts());
                frame = img.getImageFrame();
            }
            waitKey(0);
        }
    double rms=cv::calibrateCamera(objpoints, imgpoints,Size(frame.rows,frame.cols), cameraMatrix, distCoeffs, Rotation, Translation);
    cout<< rms<<endl;
    SaveFile("/home/user/Desktop/CaliCam/src/piccali/config/Output.yaml",
                Imgpathlist,
                frame,
                cameraMatrix,
                distCoeffs,
                Rotation,
                Translation,
                rms
                );
    }
    return 0;
}
