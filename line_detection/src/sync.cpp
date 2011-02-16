#include <iostream>
#include <sstream>

#include <opencv/cv.h>
#include <opencv/highgui.h>

static std::string const ext   = "png";

static double const cam_width  = 640;
static double const cam_height = 480;
static double const cam_fps    = 15;

int main(int argc, char **argv)
{
	std::string prefix;
	int id_l, id_r, delay, n;

	if (argc <= 5) {
		std::cerr << "err: incorrect number of arguments\n"
		          << "usage: ./sync-test <id l> <id r> <ms> <#> <prefix>"
		          << std::endl;
		return 0;
	}

	// Parse command-line arguments.
	std::stringstream ss;
	ss.clear(); ss.str(argv[1]); ss >> id_l;
	ss.clear(); ss.str(argv[2]); ss >> id_r;
	ss.clear(); ss.str(argv[3]); ss >> delay;
	ss.clear(); ss.str(argv[4]); ss >> n;
	prefix = argv[5];

	cv::VideoCapture cam_l(1);
	cv::VideoCapture cam_r(2);

	// Set the frame rate and resolution of the cameras to identical values.
#if 0
	cam_l.set(CV_CAP_PROP_FRAME_WIDTH,  cam_width);
	cam_l.set(CV_CAP_PROP_FRAME_HEIGHT, cam_height);
	cam_l.set(CV_CAP_PROP_FPS, cam_fps);

	cam_r.set(CV_CAP_PROP_FRAME_WIDTH,  cam_width);
	cam_r.set(CV_CAP_PROP_FRAME_HEIGHT, cam_height);
	cam_r.set(CV_CAP_PROP_FPS, cam_fps);
#endif

	for (int i = 0; i < n; ++i) {
		cv::Mat img_l, img_r;
		cam_l >> img_l;
		cam_r >> img_r;

		// Guarantee that both images have the same resolution so later code
		// does not break.
		int rows = img_l.rows;
		int cols = img_l.cols;

		if (img_l.rows != img_r.rows || img_l.cols != img_r.cols) {
			std::cerr << "err: both frames must have the same resolution" << std::endl;
			return 1;
		} else if (img_l.type() != CV_8UC3 || img_r.type() != CV_8UC3) {
			std::cerr << "err: frames must be 8-bit RGB images" << std::endl;
			return 1;
		}

		// Place both images side-by-side in a single frame.
		cv::Mat both(rows, 2 * cols, CV_8UC3);
		cv::Mat both_l = both(cv::Range(0, rows), cv::Range(0, cols));
		cv::Mat both_r = both(cv::Range(0, rows), cv::Range(cols, 2 * cols));
		img_l.copyTo(both_l);
		img_r.copyTo(both_r);

		// Write the combined images to a file.
		std::stringstream ss;
		ss << prefix << i << "." << ext;
		cv::imwrite(ss.str(), both);

		cv::waitKey(delay);
	}
	return 0;
}
