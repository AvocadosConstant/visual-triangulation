#ifndef FINAL_HPP
#define FINAL_HPP

#include <opencv2/core.hpp>
#include <vector>
#include <utility>

using _Point = cv::Point2f;
using segment_list = std::vector<std::pair<_Point, _Point>>;
using _triangle = std::tuple<_Point, _Point, _Point>;

cv::Mat canny(const cv::Mat& img);
std::vector<_Point> gen_points(const cv::Mat& img, unsigned numPts);
cv::Mat draw_img(const segment_list&, cv::Size size);
cv::Mat draw_img(const std::vector<_Point>& points, const cv::Size size);
long point_dist(const _Point& a, const _Point& b);
_Point find_centroid(const std::vector<_Point>& points);
double cross(const _Point& p1, const _Point& p2, const _Point& p3);
inline double area_tri(const _triangle&);
// Returns the opposite of what LOP actually returns
// Utilizes circumcircle test for swap
bool LOP(const _Point& p1, const _Point& p2, const _Point& p3, const _Point& p4);
_Point get_unique(const _triangle& t, _Point p1, _Point p2);

/* Triangulation */
namespace tri {
    void basic_alg(cv::Mat& img, std::vector<_Point>& points);
    segment_list radial(const cv::Mat& img, std::vector<_Point>& points);
}

#endif /* FINAL_HPP */
