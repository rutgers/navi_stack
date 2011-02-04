#include "opencv2/opencv.hpp"
#include "exports.hpp"
#include "devmem2d.hpp"

namespace bm
{
    class CVGPUBM_EXPORTS TempStereoBM_GPU
    {
        public:                                    
            enum { BASIC_PRESET = 0, PREFILTER_XSOBEL = 1 };
            enum { DEFAULT_NDISP = 64, DEFAULT_WINSZ = 19 };

            TempStereoBM_GPU();    
            TempStereoBM_GPU(int preset, int ndisparities = DEFAULT_NDISP, int winSize = DEFAULT_WINSZ);
            ~TempStereoBM_GPU();

            void operator() ( const cv::Mat& left, const cv::Mat& right, cv::Mat& disparity);

            //no acyn version here
            //void operator() ( const cv::Mat& left, const cv::Mat& right, cv::Mat& disparity, Stream& stream);
            
            //! Some heuristics that tries to estmate
            // if current GPU will be faster then CPU in this algorithm.
            // It queries current active device.
            static bool checkIfGpuCallReasonable();

            int ndisp;
            int winSize;
            int preset;

            // If avergeTexThreshold  == 0 => post procesing is disabled
            // If avergeTexThreshold != 0 then disparity is set 0 in each point (x,y) where for left image
            // SumOfHorizontalGradiensInWindow(x, y, winSize) < (winSize * winSize) * avergeTexThreshold
            // i.e. input left image is low textured.
            float avergeTexThreshold;
        private:

            void alloc(const cv::Size& sz);
            void release();

            DevMem2D_<unsigned int> minSSD;
            DevMem2D leBuf, riBuf;
            DevMem2D leBuf_solbel, riBuf_solbel;
            DevMem2D disp;

            bool allocated;
    };

    //! Speckle filtering - filters small connected components on diparity image.
    //! It sets pixel (x,y) to newVal if it coresponds to small CC with size < maxSpeckleSize.
    //! Threshold for border between CC is diffThreshold;
    CVGPUBM_EXPORTS void filterSpeckles( cv::Mat& img, uchar newVal, int maxSpeckleSize, uchar diffThreshold, cv::Mat& buf);
}
