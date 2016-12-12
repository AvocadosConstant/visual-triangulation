#ifndef _GUI_H
#define _GUI_H

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <utility>
#include <vector>

cv::Mat draw_points(cv::Mat, std::vector<cv::Point2f>, int);
std::vector<cv::Point2f> detect_corners_shi_tomasi(cv::Mat, int);

#endif
