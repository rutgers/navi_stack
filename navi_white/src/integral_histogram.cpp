#include <iostream>
#include "integral_histogram.hpp"

IntegralHistogram::IntegralHistogram(int bins_hue, int bins_sat)
	: m_bins_hue(bins_hue),
	  m_bins_sat(bins_sat),
	  m_rows(0),
	  m_cols(0)
{}

void IntegralHistogram::GetPatch(cv::Rect patch, cv::MatND &dst)
{
	CV_Assert(m_bins_hue > 0 && m_bins_sat > 0);
	CV_Assert(0 <= patch.y && patch.y + patch.height < m_rows);
	CV_Assert(0 <= patch.x && patch.x + patch.width  < m_cols);

	int const dims[] = { m_bins_hue, m_bins_sat };
	dst.create(2, dims, CV_32F);

	for (int bin_hue = 0; bin_hue < m_bins_hue; ++bin_hue)
	for (int bin_sat = 0; bin_sat < m_bins_sat; ++bin_sat) {
		cv::Mat &img_int = m_bins[bin_hue][bin_sat];
		float int_tl = img_int.at<float>(patch.y,                patch.x              );
		float int_tr = img_int.at<float>(patch.y,                patch.x + patch.width);
		float int_br = img_int.at<float>(patch.y + patch.height, patch.x + patch.width);
		float int_bl = img_int.at<float>(patch.y + patch.height, patch.x              );
		dst.at<float>(bin_hue, bin_sat) = int_br - int_bl - int_tr + int_tl;
	}
}

void IntegralHistogram::LoadImage(cv::Mat const &img)
{
	CV_Assert(img.type() == CV_8UC3);
	CV_Assert(m_bins_hue > 0 && m_bins_sat > 0);

	// Allocate an integral image for each bin.
	m_bins.resize(m_bins_hue);
	for (int bin_hue = 0; bin_hue < m_bins_hue; ++bin_hue) {
		m_bins[bin_hue].resize(m_bins_sat);
	}
	m_rows = img.rows;
	m_cols = img.cols;

	// Extract the HS-plane from the source BGR image.
	std::vector<cv::Mat> dims;
	cv::Mat hsv;
	cv::cvtColor(img, hsv, CV_BGR2HSV);
	cv::split(hsv, dims);
	cv::Mat &hue = dims[0];
	cv::Mat &sat = dims[1];

	// Compute the integral image of each bin.
	cv::Mat trash;

	for (int bin_hue = 0; bin_hue < m_bins_hue; ++bin_hue)
	for (int bin_sat = 0; bin_sat < m_bins_sat; ++bin_sat) {
		// Mask of pixels with the correct hue.
		float hue_min = ((bin_hue + 0) * 255) / m_bins_hue;
		float hue_max = ((bin_hue + 1) * 255) / m_bins_hue;
		cv::Mat img_hue_lo;
		cv::Mat img_hue_hi;
		cv::Mat img_hue;

		cv::threshold(hue, img_hue_lo, hue_min, 255, cv::THRESH_BINARY);
		cv::threshold(hue, img_hue_hi, hue_max, 255, cv::THRESH_BINARY_INV);
		cv::min(img_hue_lo, img_hue_hi, img_hue);

		// Mask of pixels with the correct saturation.
		float sat_min = ((bin_sat + 0) * 255) / m_bins_sat;
		float sat_max = ((bin_sat + 1) * 255) / m_bins_sat;
		cv::Mat img_sat_lo;
		cv::Mat img_sat_hi;
		cv::Mat img_sat;

		cv::threshold(sat, img_sat_lo, sat_min, 255, cv::THRESH_BINARY);
		cv::threshold(sat, img_sat_hi, sat_max, 255, cv::THRESH_BINARY_INV);
		cv::min(img_sat_lo, img_sat_hi, img_sat);

		// Pixels inside the bin (i.e. correct hue and saturation).
		cv::Mat img_bin;
		cv::min(img_hue, img_sat, img_bin);
		cv::integral(img_bin, m_bins[bin_hue][bin_sat], trash, CV_32FC1);
	}
}

// Half-implemented integral calculation code.
void IntegralHistogram::MatchPatches(cv::Mat src, cv::Mat &dst,
                                     cv::MatND needle, cv::Size window, int method)
{
	int const xh = window.width  / 2;
	int const yh = window.height / 2;
	int const x1 = src.cols - xh - 1;
	int const y1 = src.rows - yh - 1;

	// Pre-allocate the destination image.
	dst.create(src.rows, src.cols, CV_32FC1);
	dst.setTo(0.0);

	// Extract the HS-plane from the source BGR image.
	std::vector<cv::Mat> dims;
	cv::Mat hsv;
	cv::cvtColor(src, hsv, CV_BGR2HSV);
	cv::split(hsv, dims);
	cv::Mat &hue = dims[0];
	cv::Mat &sat = dims[1];

	// Seed the recursion with the histogram centered at pixel (0, 0).
	cv::Rect seed_rect(cv::Point(xh, yh), window);
	cv::Mat arrays[] = { hsv(seed_rect) };
	int chans[] = { 0, 1 };                   // hue and sat channels
	int sizes[] = { m_bins_hue, m_bins_sat }; // bins per channel
	float const range[]   = { 0, 255 };       // 8-bit channel depth
	float const *ranges[] = { range, range };
	cv::MatND hist, seed;
	cv::calcHist(arrays, 1, chans, cv::Mat(), hist, 2, sizes, ranges, true, false);

	for (int y = 0; y < y1; ++y) {
		// Vertical incremental update.
		if (y > 0) {
			seed.copyTo(hist);
			for (int dx = -xh; dx <= +xh; ++dx) {
				// TODO: Modify the array bounds instead of checking in here.
				bool in_x = 0 <= xh + dx    && xh + dx < src.cols;
				bool in_y = 0 <= y - yh - 1 && y  + yh < src.rows;
				if (!in_x || !in_y) continue;

				int bin_old_hue = hue.at<uint8_t>(y - yh - 1, xh + dx) * m_bins_hue / 255;
				int bin_old_sat = sat.at<uint8_t>(y - yh - 1, xh + dx) * m_bins_sat / 255;
				int bin_new_hue = hue.at<uint8_t>(y + yh, xh + dx) * m_bins_hue / 255;
				int bin_new_sat = sat.at<uint8_t>(y + yh, xh + dx) * m_bins_sat / 255;
				--hist.at<float>(bin_old_hue, bin_old_sat);
				++hist.at<float>(bin_new_hue, bin_new_sat);
			}
		}

		for (int x = 0; x < x1; ++x) {
			// Horizontal incremental update.
			if (x > 0) {
				for (int dy = -yh; dy <= +yh; ++dy) {
					// TODO: Modify the array bounds instead of checking in here.
					bool in_x = 0 <= x - xh - 1 && x + xh < src.cols;
					bool in_y = 0 <= y + dy     && y + dy < src.rows;
					if (!in_x || !in_y) continue;

					int bin_old_hue = hue.at<uint8_t>(y + dy, x - xh - 1) * m_bins_hue / 255;
					int bin_old_sat = sat.at<uint8_t>(y + dy, x - xh - 1) * m_bins_sat / 255;
					int bin_new_hue = hue.at<uint8_t>(y + dy, x + xh) * m_bins_hue / 255;
					int bin_new_sat = sat.at<uint8_t>(y + dy, x + xh) * m_bins_sat / 255;
					--hist.at<float>(bin_old_hue, bin_old_sat);
					++hist.at<float>(bin_new_hue, bin_new_sat);
				}
			}
			// Keep track of the first pixel for the vertical update.
			else {
				hist.copyTo(seed);
			}

			// Correlation of this patch with the needle.
			dst.at<float>(y, x) = cv::compareHist(hist, needle, method);
		}

	}

}
