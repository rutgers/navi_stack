#pragma warning (disable : 4996)

#include "stereo_gpu.hpp"
#include "opencv2/opencv.hpp"
#include <iostream>

using namespace std;
using namespace cv;


int main( int argc, char** argv )
{
    Mat l = cv::imread("../aloe_L.png", 0);
    Mat r = cv::imread("../aloe_R.png", 0);
    
    if (l.empty())
      return cout << "Unable to load aloe_L.png!" << endl, -1;

    if (r.empty())
      return cout << "Unable to load aloe_R.png!" << endl, -1;

    const int ndisp = 168;
    const int winSize = 11;

    //bm::TempStereoBM_GPU matcher(bm::TempStereoBM_GPU::BASIC_PRESET, ndisp, winSize);
    bm::TempStereoBM_GPU matcher(bm::TempStereoBM_GPU::PREFILTER_XSOBEL, ndisp, winSize);

    Mat disp, buf;
    matcher(l, r, disp); // first call is slow due to Cuda initialization. You may init it manually by calling cudaSetDevice.
    bm::filterSpeckles( disp, 0, 60, 2, buf);

    TickMeter tm;
    const int count = 10;
    
        
    bool res = bm::TempStereoBM_GPU::checkIfGpuCallReasonable();    
    if (res)
    {
        cout << "Oh, good GPU! Please start block matching." << endl;
    }
    else
    {
        cout << "Are you joking? You'd better to upgrade your GPU before stero matching" << endl;        
    }
    

    tm.start();
    for(int i = 0; i < count; ++i)
    {
        matcher(l, r, disp);
        
        //In this package there are not acync mechanisms.
        //So your can't start GPU matcher for next frame and speckles filtering for prev in parrallel.
        //filterSpeckles is on CPU and not very fast.
        //bm::filterSpeckles( disp, 0, 60, 2, buf); 
    }
    tm.stop();

    cout << "FPS = " << count/tm.getTimeSec() << endl;

    { Mat tmp; resize(disp, tmp, Size(800, 600)); disp = tmp; }
    imshow("D", disp); waitKey();
}
