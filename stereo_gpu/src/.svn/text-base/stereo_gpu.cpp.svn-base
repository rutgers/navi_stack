#pragma warning (disable : 4996)

#include<iostream>

#include "cuda_runtime_api.h"
#include "safe_call.hpp"
#include "stereo_gpu.hpp"

using namespace std;
using namespace cv;
using namespace bm;

namespace bm
{
    extern "C" void stereoBM_GPU(const DevMem2D& left, const DevMem2D& right, const DevMem2D& disp, int ndisp, int winsz, const DevMem2D_<unsigned int>& minSSD_buf, const cudaStream_t & stream);
    extern "C" void prefilter_xsobel(const DevMem2D& input, const DevMem2D& output, int prefilterCap /*= 31*/, const cudaStream_t & stream);
    extern "C" void postfilter_textureness(const DevMem2D& input, int winsz, float avgTexturenessThreshold, const DevMem2D& disp, const cudaStream_t & stream);
}

extern "C" void error(const char *error_string, const char *file, const int line, const char *func)
{                       
    cv::error( cv::Exception(CV_GpuApiCallError, error_string, func, file, line) );
}


template<class T> void upload(const DevMem2D_<T>& dst, const Mat& src)
{
    CV_DbgAssert(dst.cols == src.cols && dst.rows == src.rows && dst.elemSize() == src.elemSize());
    cudaSafeCall( cudaMemcpy2D(dst.ptr, dst.step, src.data, src.step, dst.cols * dst.elemSize(), dst.rows, cudaMemcpyHostToDevice) );
}

template<class T> void download(const Mat& dst, const DevMem2D_<T>& src)
{
    CV_DbgAssert(dst.cols == src.cols && dst.rows == src.rows && dst.elemSize() == src.elemSize());    
    cudaSafeCall( cudaMemcpy2D(dst.data, dst.step, src.ptr, src.step, src.cols * src.elemSize(), src.rows, cudaMemcpyDeviceToHost) );
}

bool bm::TempStereoBM_GPU::checkIfGpuCallReasonable()
{
    int count;
    cudaSafeCall( cudaGetDeviceCount(&count) );
    if (0 == count)
        return false;

    int device;
    cudaSafeCall( cudaGetDevice(&device) );
    
    cudaDeviceProp prop;    
    cudaSafeCall( cudaGetDeviceProperties( &prop, device) );
        
    if (prop.major > 1 || prop.multiProcessorCount > 16)
        return true;

    return false;
}

const float defaultAvgTexThreshold = 3;

bm::TempStereoBM_GPU::TempStereoBM_GPU() 
    : ndisp(DEFAULT_NDISP), winSize(DEFAULT_WINSZ), preset(BASIC_PRESET), 
    avergeTexThreshold(defaultAvgTexThreshold), allocated(false)  {}

bm::TempStereoBM_GPU::TempStereoBM_GPU(int _preset, int _ndisp, int _winSize) 
    : ndisp(_ndisp), winSize(_winSize), preset(_preset), 
    avergeTexThreshold(defaultAvgTexThreshold), allocated(false)  {}

bm::TempStereoBM_GPU::~TempStereoBM_GPU() { release(); }

void bm::TempStereoBM_GPU::release()
{          
    if (!allocated)
        return;

    cudaSafeCall( cudaFree( minSSD.ptr ) );
    cudaSafeCall( cudaFree( disp.ptr ) );

    cudaSafeCall( cudaFree( leBuf.ptr ) );
    cudaSafeCall( cudaFree( riBuf.ptr ) );

    cudaSafeCall( cudaFree( leBuf_solbel.ptr ) );
    cudaSafeCall( cudaFree( riBuf_solbel.ptr ) );

    allocated = false;
}

void bm::TempStereoBM_GPU::alloc(const Size& sz)
{
    if (allocated && minSSD.cols == sz.width && minSSD.rows == sz.height) 
        return;
    
    release();        

    disp.cols = minSSD.cols = leBuf.cols = riBuf.cols = leBuf_solbel.cols = riBuf_solbel.cols = sz.width;
    disp.rows = minSSD.rows = leBuf.rows = riBuf.rows = leBuf_solbel.rows = riBuf_solbel.rows = sz.height;

    cudaSafeCall( cudaMallocPitch((void**)&minSSD.ptr, &minSSD.step, sz.width * minSSD.elemSize(), sz.height) );
    cudaSafeCall( cudaMallocPitch((void**)&disp.ptr, &disp.step, sz.width, sz.height) );

    cudaSafeCall( cudaMallocPitch((void**)&leBuf.ptr, &leBuf.step, sz.width, sz.height) );
    cudaSafeCall( cudaMallocPitch((void**)&riBuf.ptr, &riBuf.step, sz.width, sz.height) );

    cudaSafeCall( cudaMallocPitch((void**)&leBuf_solbel.ptr, &leBuf_solbel.step, sz.width, sz.height) );
    cudaSafeCall( cudaMallocPitch((void**)&riBuf_solbel.ptr, &riBuf_solbel.step, sz.width, sz.height) );    

    allocated = true;
}

void bm::TempStereoBM_GPU::operator() ( const cv::Mat& left, const cv::Mat& right, cv::Mat& disparity)
{
    const int max_supported_ndisp = 1 << (sizeof(unsigned char) * 8);
    CV_Assert(0 < ndisp && ndisp <= max_supported_ndisp);
    CV_Assert(ndisp % 8 == 0);
    CV_Assert(winSize % 2 == 1);

    CV_Assert(left.size() == right.size() && left.type() == CV_8UC1 && right.type() == CV_8UC1);   

    disparity.create(left.size(), CV_8U);

    alloc(left.size());    

    upload(preset == PREFILTER_XSOBEL ? leBuf_solbel : leBuf,  left);
    upload(preset == PREFILTER_XSOBEL ? riBuf_solbel : riBuf, right);

    if (preset == PREFILTER_XSOBEL)
    {           
		bm::prefilter_xsobel(leBuf_solbel, leBuf, 31, 0);
        bm::prefilter_xsobel(riBuf_solbel, riBuf, 31, 0);        
    }
    

    bm::stereoBM_GPU(leBuf, riBuf, disp, ndisp, winSize, minSSD, 0);

    if (avergeTexThreshold != 0)
        bm::postfilter_textureness(leBuf, winSize, avergeTexThreshold, disp, 0);    

    download(disparity, disp);
}



typedef Point_<short> Point2s;

void bm::filterSpeckles( Mat& img, uchar newVal, int maxSpeckleSize, uchar maxDiff, Mat& _buf)
{
    int MaxD = 1024;
    int WinSz = 64;

    int bufSize0 = (MaxD + 2)*sizeof(int) + (img.rows+WinSz+2)*MaxD*sizeof(int) +
        (img.rows + WinSz + 2)*sizeof(int) +
        (img.rows+WinSz+2)*MaxD*(WinSz+1)*sizeof(uchar) + 256;
    int bufSize1 = (img.cols + 9 + 2) * sizeof(int) + 256;
    int bufSz = max(bufSize0 * 1, bufSize1 * 2);

    _buf.create(1, bufSz, CV_8U);

    CV_Assert( img.type() == CV_8U );

    int width = img.cols, height = img.rows, npixels = width*height;
    size_t bufSize = npixels*(int)(sizeof(Point2s) + sizeof(int) + sizeof(uchar));
    if( !_buf.isContinuous() || !_buf.data || _buf.cols*_buf.rows*_buf.elemSize() < bufSize )
        _buf.create(1, bufSize, CV_8U);

    uchar* buf = _buf.data;
    int i, j, dstep = img.step/sizeof(uchar);
    int* labels = (int*)buf;
    buf += npixels*sizeof(labels[0]);
    Point2s* wbuf = (Point2s*)buf;
    buf += npixels*sizeof(wbuf[0]);
    uchar* rtype = (uchar*)buf;
    int curlabel = 0;

    // clear out label assignments
    memset(labels, 0, npixels*sizeof(labels[0]));

    for( i = 0; i < height; i++ )
    {
        uchar* ds = img.ptr<uchar>(i);
        int* ls = labels + width*i;

        for( j = 0; j < width; j++ )
        {
            if( ds[j] != newVal )	// not a bad disparity
            {
                if( ls[j] )		// has a label, check for bad label
                {  
                    if( rtype[ls[j]] ) // small region, zero out disparity
                        ds[j] = (uchar)newVal;
                }
                // no label, assign and propagate
                else
                {
                    Point2s* ws = wbuf;	// initialize wavefront
                    Point2s p((short)j, (short)i);	// current pixel
                    curlabel++;	// next label
                    int count = 0;	// current region size
                    ls[j] = curlabel;

                    // wavefront propagation
                    while( ws >= wbuf ) // wavefront not empty
                    {
                        count++;
                        // put neighbors onto wavefront
                        uchar* dpp = &img.at<uchar>(p.y, p.x);
                        uchar dp = *dpp;
                        int* lpp = labels + width*p.y + p.x;

                        if( p.x < width-1 && !lpp[+1] && dpp[+1] != newVal && std::abs(dp - dpp[+1]) <= maxDiff )
                        {
                            lpp[+1] = curlabel;
                            *ws++ = Point2s(p.x+1, p.y);
                        }

                        if( p.x > 0 && !lpp[-1] && dpp[-1] != newVal && std::abs(dp - dpp[-1]) <= maxDiff )
                        {
                            lpp[-1] = curlabel;
                            *ws++ = Point2s(p.x-1, p.y);
                        }

                        if( p.y < height-1 && !lpp[+width] && dpp[+dstep] != newVal && std::abs(dp - dpp[+dstep]) <= maxDiff )
                        {
                            lpp[+width] = curlabel;
                            *ws++ = Point2s(p.x, p.y+1);
                        }

                        if( p.y > 0 && !lpp[-width] && dpp[-dstep] != newVal && std::abs(dp - dpp[-dstep]) <= maxDiff )
                        {
                            lpp[-width] = curlabel;
                            *ws++ = Point2s(p.x, p.y-1);
                        }

                        // pop most recent and propagate
                        // NB: could try least recent, maybe better convergence
                        p = *--ws;
                    }

                    // assign label type
                    if( count <= maxSpeckleSize )	// speckle region
                    {
                        //printf("count = %d\n", count);
                        rtype[ls[j]] = 1;	// small region label
                        ds[j] = (uchar)newVal;
                    }
                    else
                    {
                        //printf("count = %d\n", count);
                        rtype[ls[j]] = 0;	// large region label
                    }
                }
            }
        }
    }
}    
      