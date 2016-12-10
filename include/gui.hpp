#ifndef _GUI_H
#define _GUI_H

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <utility>
#include <vector>

void compress(cv::Mat &m, const int size);
void exportImage(cv::Mat &m, const int size, std::string filename);
void importImage(cv::Mat &m, std::string filename);

#endif
