#include <istream>
#include <sstream>
#include <string>
#include <vector>
#include <stdint.h>
#include <opencv/cv.h>

#include "csv.hpp"

#include <iostream>

float ParseFloat(std::string raw)
{
	std::stringstream ss;
	ss.str(raw);
	float value;
	ss >> value;
	return value;
}

void TokenizeRow(std::istream &stream, char delim, std::vector<std::string> &data)
{
	std::string line;
	std::getline(stream, line);
	std::stringstream ss(line);

	while (ss.good()) {
		std::string token;
		std::getline(ss, token, delim);
		data.push_back(token);
	}
}

bool Parse(std::istream &stream, cv::Mat &features, cv::Mat &labels, char delim)
{
	std::vector<float> vec_features;
	std::vector<float> vec_labels;
	size_t n = 1;

	for (;;) {
		std::vector<std::string> tokens;
		TokenizeRow(stream, delim, tokens);

		if (stream.good()) {
			n = tokens.size() - 1;

			// All but the last column are 8-bit features.
			for (size_t i = 0; i < n; ++i) {
				float feature = ParseFloat(tokens[i]);
				vec_features.push_back(feature);
			}

			// Last column is the label.
			float label = ParseFloat(tokens[n]);
			vec_labels.push_back(label);
		} else if (stream.eof()) {
			break;
		} else {
			return false;
		}
	}

	features = cv::Mat(vec_features, true).reshape(0, vec_features.size() / n);
	labels   = cv::Mat(vec_labels, true);
	return true;
}
