#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include "Calibration.cpp"
#include <iostream>
#include <string>
using namespace cv;
using namespace std;

#define SQUAESIZE 25
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




    
int main(int argc , char** argv){

    vector<string> Imgpathlist;
    LoadFile("/home/user/Desktop/webcam/src/picpass/config/InputSetting.xml",Imgpathlist);
    for(int i =0; i < Imgpathlist.size();i++){
        /*---------------Display Images------------------------*/
        Calibration cali(Imgpathlist[i],W_CHECKBOX,H_CHECKBOX);
        cali.RunCalibration();
        cali.PrintIntrinsicParam();
        cali.PrintExtrinsicParam();
        waitKey(0); 
    }   
    return 0;
}