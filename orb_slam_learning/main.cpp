#include <iostream>
#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<opencv2/core/core.hpp>

//#include</home/sc/Code/ORB_SLAM2-master/include/System.h>
#include<System.h>

using namespace std;
using namespace cv;
int main(int argc,char ** argv) {
    vector<string> vstrImageFileNamesRgb;
    vector<string> vstrImageFilenamesD;
    vector<double> vTimestamps;
    string strAssociationFileName =string(argv[4]);
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true);

    std::cout << "Hello, World!" << std::endl;
    return 0;
}