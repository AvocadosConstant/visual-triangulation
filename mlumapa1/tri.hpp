#ifndef FINAL_HPP
#define FINAL_HPP

#include <opencv2/core.hpp>
#include <vector>
#include <utility>

using segment_list = std::vector<std::pair<cv::Point2i, cv::Point2i>>;

cv::Mat canny(const cv::Mat& img);
std::vector<cv::Point2i> gen_points(const cv::Mat& img, unsigned numPts);
cv::Mat draw_img(const segment_list&, cv::Size size);
long point_dist(const cv::Point2i& a, const cv::Point2i& b);
cv::Point2i find_centroid(const std::vector<cv::Point2i>& points);

// Returns the opposite of what LOP actually returns
// Utilizes circumcircle test for swap
bool LOP(const cv::Point2i& p1, const cv::Point2i& p2, const cv::Point2i& p3, const cv::Point2i& p4);

/* Triangulation */
namespace tri {
    void basic_alg(cv::Mat& img, std::vector<cv::Point2i>& points);
    segment_list radial(const cv::Mat& img, std::vector<cv::Point2i>& points);
}

#endif /* FINAL_HPP */
