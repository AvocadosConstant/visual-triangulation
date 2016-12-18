#ifndef FINAL_HPP
#define FINAL_HPP

#include <opencv2/core.hpp>
#include <vector>
#include <utility>

using _Point = cv::Point2f;
using segment_list = std::vector<std::pair<_Point, _Point>>;
using _triangle = std::tuple<_Point, _Point, _Point>;

/* Unrelated to triangulation */
cv::Mat draw_img(const segment_list&, cv::Size size);
cv::Mat draw_img(const segment_list&, const cv::Mat& img);
cv::Mat draw_img(const std::vector<_Point>& points, const cv::Size size);

/* Triangulation auxilliaries */
long point_dist(const _Point& a, const _Point& b);
_Point find_centroid(const std::vector<_Point>& points);
double cross(const _Point& p1, const _Point& p2, const _Point& p3);
inline double dot(const _Point& p1, const _Point& p2, const _Point& p3);
double dot(const _Point& p, const _Point& pivot);
inline double area_tri(const _triangle&);
inline double area_tri(const _Point& p1, const _Point& p2, const _Point& p3);

// If > 0, left bend. == 0, collinear. < 0, right bend. p1 is base point of
// vectors p1p2 and p1p3.
inline long compute_direction(const _Point& p1, const _Point& p2, const _Point& p3);

// Returns the opposite of what LOP actually returns
// Utilizes circumcircle test for swap
bool LOP(const _Point& p1, const _Point& p2, const _Point& p3, const _Point& p4);
_Point get_unique(const _triangle& t, _Point p1, _Point p2);

// Convex hull (Graham scan)
segment_list compute_hull(const std::vector<_Point>& points);

/* Triangulation */
namespace tri {
    segment_list radial(std::vector<_Point>& points);
}

#endif /* FINAL_HPP */
