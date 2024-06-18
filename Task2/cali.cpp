#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/persistence.hpp>

#include "Calibration.cpp"
#include <iostream>
#include <string>
using namespace cv;
using namespace std;

//#define SQUAESIZE 25
#define W_CHECKBOX 6
#define H_CHECKBOX 8


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
bool SaveFile(string filepath_name,string imgpathname,Calibration cali)
{
    FileStorage fs;
    fs.open(filepath_name, FileStorage::APPEND |FileStorage::FORMAT_YAML);
    if (!fs.isOpened()){
        cerr << "Failed to open " << filepath_name << endl;
        return false;
    }
    fs<<"Imagepath"<<imgpathname;
    fs<<"Image Width"<<cali.getFramesize().width;
    fs<<"Image Height"<<cali.getFramesize().height;
        fs<<"Intrinsic-Camera-Parameter"<<"{";
        fs<<"Focal x"<<cali.getCameraMatrix().at<double>(0,0);
        fs<<"Focal y"<<cali.getCameraMatrix().at<double>(1,1);
        fs<<"px cx "<<cali.getCameraMatrix().at<double>(0,2);
        fs<<"py cy"<<cali.getCameraMatrix().at<double>(1,2);
        fs<<"}";
        fs<<"Distortion coefficients"<<"{";
        fs<< "k1" << cali.getDistCoeffs().at<double>(0,0);
        fs<< "k2" << cali.getDistCoeffs().at<double>(0,1);
        fs<< "p1" << cali.getDistCoeffs().at<double>(0,2);
        fs<< "p2" << cali.getDistCoeffs().at<double>(0,3);
        fs<< "k3" << cali.getDistCoeffs().at<double>(0,4);
        fs<<"}";
    fs<<"Extrinsic-Camera-Parameter"<<"{";// 9x9 matrix
    fs<<"Translation"<<cali.getTranslation();
    fs<<"Rotation"<<cali.RealRotation(cali.getRotation());
    fs<<"}";
    fs.release();
    return true;
}



    
int main(int argc , char** argv){

    vector<string> Imgpathlist;
    LoadFile("/home/user/Desktop/CaliCam/src/piccali/config/InputSetting.xml",Imgpathlist);
    for(int i =0; i < Imgpathlist.size();i++){
        /*---------------Display Images------------------------*/
        Calibration cali(Imgpathlist[i],W_CHECKBOX,H_CHECKBOX);
        cali.RunCalibration();
        cali.PrintIntrinsicParam();
        cali.PrintExtrinsicParam();
        bool status = SaveFile("/home/user/Desktop/CaliCam/src/piccali/config/Output.yaml",Imgpathlist[i],cali);
        if (status == false){
            return 1;
        }
        waitKey(0); 
    }   
    return 0;
}
