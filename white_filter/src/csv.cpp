#include <istream>
#include <sstream>
#include <string>
#include <vector>
#include <stdint.h>
#include <opencv/cv.h>

#include "csv.hpp"

uint8_t ParseByte(std::string raw)
{
	std::stringstream ss;
	ss.str(raw);

	int value;
	ss >> value;
	return (uint8_t)value;
}

void TokenizeRow(std::istream &stream, char delim, std::vector<std::string> data)
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

bool Parse(std::istream &stream, cv::Mat &features, cv::Mat &labels,
           char delim, std::string label_true)
{
	std::vector<uint8_t> vec_features;
	std::vector<uint8_t> vec_labels;
	size_t n = 1;

	for (;;) {
		std::vector<std::string> tokens;
		TokenizeRow(stream, delim, tokens);

		if (stream.good()) {
			// All but the last column are 8-bit features.
			n = tokens.size() - 1;
			for (size_t i = 0; i < n; ++i) {
				uint8_t feature = ParseByte(tokens[i]);
				vec_features.push_back(feature);
			}

			// Last column is the label.
			uint8_t label = (label_true == tokens[n]);
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
