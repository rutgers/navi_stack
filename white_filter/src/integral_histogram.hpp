#ifndef INTEGRAL_HISTOGRAM_HPP_
#define INTEGRAL_HISTOGRAM_HPP_

#include <vector>
#include <opencv/cv.h>

class IntegralHistogram {
public:
	IntegralHistogram(int bins_hue, int bins_sat);
	void LoadImage(cv::Mat const &img);
	void GetPatch(cv::Rect patch, cv::MatND &dst);
	void MatchPatches(cv::Mat src, cv::Mat &dst, cv::MatND needle, cv::Size window, int method);

private:
	int m_bins_hue, m_bins_sat;
	int m_rows, m_cols;
	std::vector<std::vector<cv::Mat> > m_bins;
};

#endif
