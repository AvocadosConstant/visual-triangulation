#include "tri.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <queue>
#include <random>
#include <tuple>
#include <unordered_map>
#include <vector>

using _Point = cv::Point2f;
using _segment = std::pair<_Point, _Point>;
using segment_list = std::vector<_segment>;
using _triangle = std::tuple<_Point, _Point, _Point>;
using triangle_list = std::vector<_triangle>;

cv::Mat canny(const cv::Mat& img) {
    cv::Mat dummy;
    double highThresh = cv::threshold(img, dummy, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
    cv::Canny(img, dummy, 0.5 * highThresh, highThresh);
    return dummy.clone();
}

std::vector<_Point> gen_points(const cv::Mat& img, unsigned numPts) {
    std::vector<_Point> points;
    for(int row = 0; row < img.rows; ++row)
        for(int col = 0; col < img.cols; ++col) {
            if(img.at<uchar>(row,col) == 255) {
                points.emplace_back(row, col);
            }
        }

    //std::random_device rd;
    std::shuffle(points.begin(), points.end(), std::mt19937(69));
    return std::vector<_Point>(points.begin(), points.begin() + numPts);
}

cv::Mat draw_img(const segment_list& segments, const cv::Size size) {
    cv::Mat img = cv::Mat::zeros(size, CV_8UC1);
    for(auto s : segments) {
        cv::line(img, std::get<0>(s), std::get<1>(s), cv::Scalar(255), 1, CV_AA);
        //cv::circle(img, points[i], 1, cv::Scalar(255,255,255), 2);
    }

    return img.clone();
}

cv::Mat draw_img(const std::vector<_Point>& points, const cv::Size size) {
    cv::Mat img = cv::Mat::zeros(size, CV_8UC1);
    for(auto p : points) {
        cv::circle(img, p, 1, cv::Scalar(255,255,255), 1);
    }

    return img.clone();
}

long point_dist(const _Point& a, const _Point& b) {
    return (a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y);
}

_Point find_centroid(const std::vector<_Point>& points) {
    long x = 0; long y = 0;
    for(auto p : points) { x += p.x; y += p.y; }
    if(points.size() > 0) { x /= points.size(); y /= points.size(); }
    std::cout << "X: " << x << ", Y: " << y << std::endl;

    _Point centroid(x,y);
    _Point retPt;
    long minDist = std::numeric_limits<long>::max();

    for(auto p : points) {
        long currentDist = point_dist(centroid,p);
        if(currentDist < minDist) {
            minDist = currentDist;
            retPt = p;
        }
    }

    return retPt;
}

double cross(const _Point& p1, const _Point& p2, const _Point& p3) {
    double sum1 = p1.x*p2.y + p2.x*p3.y + p3.x*p1.y;
    double sum2 = p2.x*p1.y + p3.x*p2.y + p1.x*p3.y;
    return std::abs((sum1 - sum2) / 2);
}

inline double area_tri(const _triangle& t) {
    return cross(std::get<0>(t), std::get<1>(t), std::get<2>(t));
}

bool LOP(const _Point& p1, const _Point& p2, const _Point& p3, const _Point& p4) {
    double sin_a = (p3.x - p2.x) * (p1.y - p2.y) - (p1.x - p2.x) * (p3.y - p2.y);
    double sin_b = (p1.x - p4.x) * (p3.y - p4.y) - (p3.x - p4.x) * (p1.y - p4.y);
    double cos_a = (p3.x - p2.x) * (p1.x - p2.x) + (p3.y - p2.y) * (p1.y - p2.y);
    double cos_b = (p1.x - p4.x) * (p3.x - p4.x) + (p1.y - p4.y) * (p3.y - p4.y);
    if(cos_a < 0 && cos_b < 0) return true;
    if(cos_a > 0 && cos_b > 0) return false;
    if(cos_a * sin_b + sin_a * cos_b < 0 ||
            -1*cos_a*sin_b + -1*sin_a*cos_b < 0) return true;
    return false;
}

_Point get_unique(const _triangle& t, _Point p1, _Point p2) {
    auto p = std::get<0>(t);
    if(p != p1 && p != p2) return p;
    p = std::get<1>(t);
    if(p != p1 && p != p2) return p;
    p = std::get<2>(t);
    return p;
}

namespace tri {
    void basic_alg(cv::Mat& img, std::vector<_Point>& points) {
        // Fill
    }

    /* Radial sweep */
    segment_list radial(const cv::Mat& img, std::vector<_Point>& points) {
        segment_list segments;

        /* Hashing function for points */
        std::function<unsigned long(_Point)> point_hasher = [](_Point p) -> unsigned long {
            return (53 + std::hash<unsigned long>{}(p.x)) * 53 + std::hash<unsigned long>{}(p.y);
        };

        /* Hashing function for segments */
        auto seg_hasher = [&point_hasher](const _segment s) -> unsigned long {
            return 53 + point_hasher(std::get<0>(s)) * point_hasher(std::get<1>(s));
        };

        /* Equality function for segments */
        auto seg_tri_eq = [](const _segment& s1, const _segment& s2) {
            auto s1_1 = std::get<0>(s1); auto s1_2 = std::get<1>(s1);
            auto s2_1 = std::get<0>(s2); auto s2_2 = std::get<1>(s2);
            return (s1_1 == s2_1 && s1_2 == s2_2) || (s1_1 == s2_2 && s1_2 == s2_1);
        };

        /* Adjacency list (using a map) */
        std::unordered_map<_Point, std::vector<_Point>, decltype(point_hasher)> adj_map(points.size(), point_hasher);

        /* Triangle list (bound to each segment) */
        std::unordered_map<_segment, triangle_list, decltype(seg_hasher), decltype(seg_tri_eq)> tri_map(points.size()*2/3, seg_hasher, seg_tri_eq);

        for(auto p : points) adj_map.emplace(p, std::vector<_Point>());

        // Find centroidal point (point near centroid)
        auto centroid = find_centroid(points);
        int quadrants[2][2] = {{0, 3}, {1, 2}};

        // Comparator function for clockwise (around centroid) sort
        auto compare_pts = [centroid, &quadrants](_Point a, _Point b) {
            a -= centroid; b -= centroid;
            int a_x = (a.x < 0) ? 1 : 0; int a_y = (a.y < 0) ? 1 : 0;
            int b_x = (b.x < 0) ? 1 : 0; int b_y = (b.y < 0) ? 1 : 0;
            int coll1 = (b.y - a.y) * (centroid.x - b.x);
            int coll2 = (centroid.y - b.y) * (b.x - a.x);
            double slope1 = 1.0 * (a.y / (a.x + 0.0001));
            double slope2 = 1.0 * (b.y / (b.x + 0.0001));

            bool retVal = false;
            if(quadrants[a_x][a_y] == quadrants[b_x][b_y]) {
                // Collinear check
                if(coll1 == coll2) {
                    auto dist1 = point_dist(a, centroid);
                    auto dist2 = point_dist(b, centroid);
                    retVal = (dist1 < dist2) ? true : false;
                }
                else retVal = slope1 < slope2;
            }
            else if(quadrants[a_x][a_y] < quadrants[b_x][b_y]) retVal = true;

            return retVal;
        };

        // Function for swapping edges in LOP
        auto seg_swap = [&adj_map](_Point p1, _Point p2, _Point p3, _Point p4) {
            auto& v1 = adj_map[p1]; auto& v2 = adj_map[p2];
            std::remove(v1.begin(), v1.end(), p2);
            std::remove(v2.begin(), v2.end(), p1);
            adj_map[p3].push_back(p4);
            adj_map[p4].push_back(p3);
        };

        auto insert_tri_map = [&tri_map](const _Point p1, const _Point p2, const _Point p3) {
            tri_map[_segment(p1,p2)].emplace_back(p1,p2,p3);
            tri_map[_segment(p2,p3)].emplace_back(p1,p2,p3);
            tri_map[_segment(p1,p3)].emplace_back(p1,p2,p3);
        };

        auto triang_eq = [](const _triangle& t1, const _triangle& t2) -> bool {
            _Point t1_points[3] = { std::get<0>(t1), std::get<1>(t1), std::get<2>(t1) };
            _Point t2_points[3] = { std::get<0>(t2), std::get<1>(t2), std::get<2>(t2) };
            bool isEqual = false;
            for(auto p1 : t1_points) {
                isEqual = false;
                for(auto p2 : t2_points) {
                    if(p1 == p2) {
                        isEqual = true;
                        break;
                    }
                }
                if(!isEqual) break;
            }

            return isEqual;
        };

        auto remove_tri = [&tri_map, &triang_eq](const _triangle& t, const _Point p1, const _Point p2) {
            auto& m = tri_map[_segment(p1,p2)];
            for(auto iter = m.begin(); iter != m.end(); ++iter) {
                if(triang_eq(t, *iter)) {
                    m.erase(iter);
                    break;
                }
            }
        };

        // Draw lines from centroid to outer points
        std::vector<_Point>& centroid_adj_list = adj_map[centroid];
        for(auto p : points) {
            segments.emplace_back(centroid, p);
            centroid_adj_list.push_back(p);
            adj_map[p].push_back(centroid);
        }

        // Order points
        std::sort(points.begin(), points.end(), compare_pts);


        // Connect lines in clockwise fashion
        for(std::size_t i = 2; i < points.size(); ++i) {
            segments.emplace_back(points[i-1], points[i]);
            adj_map[points[i-1]].push_back(points[i]);
            adj_map[points[i]].push_back(points[i-1]);
            insert_tri_map(centroid, points[i-1], points[i]);
        }

        segments.emplace_back(points.back(), points[1]);
        adj_map[points.back()].push_back(points[1]);
        adj_map[points[1]].push_back(points.back());
        insert_tri_map(centroid, points[1], points.back());

        // Make all boundaries convex edges
        for(std::size_t i = 3; i < points.size(); ++i) {
            auto dist1 = point_dist(points[i-2], centroid);
            auto dist2 = point_dist(points[i-1], centroid);
            auto dist3 = point_dist(points[i], centroid);

            if(dist1 > dist2 && dist3 > dist2) {
                segments.emplace_back(points[i-2], points[i]);
                adj_map[points[i-2]].push_back(points[i]);
                adj_map[points[i]].push_back(points[i-2]);
                insert_tri_map(points[i-2], points[i-1], points[i]);
            }
        }

        std::cout << "Centroid: " << centroid << std::endl;

        bool edgeFlip = true;
        //int nuum = 0;
        // LOP - Local Optimization Procedure
        for(int iter = 0; iter < 1; ++iter) {
        //while(edgeFlip) {
            edgeFlip = false;
            std::cout << "Size before: " << tri_map[_segment({4,95},{175,116})].size() << std::endl;
            for(auto& s : segments) {
                if(seg_tri_eq(s, _segment({4,95}, {175,116}))) {
                    std::cout << "Size: " << tri_map[s].size() << std::endl;
                }
                if(tri_map[s].size() != 2) continue;
                //if(nuum == 4) break;
                auto p1 = std::get<0>(s);
                auto p2 = std::get<1>(s);
                auto t1 = tri_map[s].at(0);
                auto t2 = tri_map[s].at(1);

                auto p3 = get_unique(t1, p1, p2);
                auto p4 = get_unique(t2, p1, p2);


                // Flip edge according to LOP and check if valid diagonal
                //if(LOP(p1, p3, p2, p4) && tri_map[_segment(p3,p4)].size() == 0 &&
                       //(area_tri(t1)+area_tri(t2) == cross(p3,p1,p4)+cross(p3,p2,p4))) {

                //if(LOP(p1, p3, p2, p4) && tri_map[_segment(p3,p4)].size() == 0) {
                if(LOP(p1, p3, p2, p4) && (area_tri(t1)+area_tri(t2) == cross(p3,p1,p4)+cross(p3,p2,p4))) {
                    //nuum++;
                    //std::cout << "Orig area: " << (area_tri(t1) + area_tri(t2)) << std::endl;
                    //std::cout << "New area: " << (cross(p3,p1,p4) + cross(p3,p2,p4)) << std::endl;
                    if(!edgeFlip) edgeFlip = true;
                    std::cout << "Flipping " << p1 << "," << p2
                        << " to " << p3 << "," << p4 << std::endl;
                    tri_map[s].clear();
                    remove_tri(t1, p1, p3);
                    remove_tri(t1, p2, p3);
                    remove_tri(t2, p1, p4);
                    remove_tri(t2, p2, p4);
                    seg_swap(p1, p2, p3, p4);
                    s = std::make_pair(p3, p4);
                    insert_tri_map(p3, p1, p4);
                    insert_tri_map(p3, p2, p4);
                }
            }
            std::cout << "End of " << iter << std::endl;
            //std::cout << "End" << std::endl;
        }

        return segments;
    }

}
