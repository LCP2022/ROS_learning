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
string getconifgsingle(FileStorage fs,string name){
    return fs[name];
}
bool LoadFile(string filepath_name ,FileStorage &outfs){
    outfs.open(filepath_name, FileStorage::READ);
    if (!outfs.isOpened()){
        cerr << "Failed to open " << filepath_name << endl;
        return false;
    }
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
    FileStorage fs;
    bool status = LoadFile("/home/user/Desktop/CaliCam/src/piccali/config/InputSetting.xml",fs);
    status=0;
    if(status){
        getconifgseq(fs,"images",Imgpathlist);
        for(int i =0; i < Imgpathlist.size();i++){
            /*---------------Display Images------------------------*/
            Calibration cali(Imgpathlist[i],W_CHECKBOX,H_CHECKBOX);
            cali.RunCalibration();
            // bool status = SaveFile("/home/user/Desktop/CaliCam/src/piccali/config/Output.yaml",Imgpathlist[i],cali);
            // if (status == false){
            //     return 1;
            // }
            waitKey(0);    
        }
    }
    else{
        string devicepath = getconifgsingle(fs,"deivce_path");
        VideoCapture cap(devicepath);
        Mat frame;
        if (!cap.isOpened()) {
        cerr << "ERROR! Unable to open camera\n";
        return -1;
        }
        while(1)
        {  
            cap.read(frame);
            // check if we succeeded
            if (frame.empty()) {
            cerr << "ERROR! blank frame grabbed\n";
            break;
            }
            imshow("Live", frame);
            int value = waitKeyEx(30);
            switch(value){
                case 27 : return 0;break;
                case 32 : Calibration cali(frame,W_CHECKBOX,H_CHECKBOX);
                          cali.RunCalibrationFrame();
                          break;
            }
        }
    }
    return 0;
}
