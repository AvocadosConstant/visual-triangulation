#ifndef _GUI_H
#define _GUI_H

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <utility>
#include <vector>

cv::Mat draw_points(cv::Mat, std::vector<cv::Point2f>, int);
std::vector<cv::Point2f> detect_corners_random_edge(cv::Mat, int);
std::vector<cv::Point2f> detect_corners_shi_tomasi(cv::Mat, int);

// Compression
void compress(cv::Mat &m, const int size, const int color);
void exportImage(cv::Mat &m, const int size, const int color, std::string filename);
void importImage(cv::Mat &m, const int color, std::string filename);

#endif
