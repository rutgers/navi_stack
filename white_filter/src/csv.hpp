#ifndef CSV_HPP_
#define CSV_HPP_

#include <istream>
#include <string>
#include <vector>
#include <opencv/cv.h>

uint8_t ParseByte(std::string raw);
void TokenizeRow(std::istream &stream, char delim, std::vector<std::string> data);
bool Parse(std::istream &stream, cv::Mat &features, cv::Mat &labels, char delim, std::string label);

#endif
