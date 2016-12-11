#include "tri.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <random>
#include <unordered_map>
#include <vector>

using segment_list = std::vector<std::pair<cv::Point2i, cv::Point2i>>;

cv::Mat canny(const cv::Mat& img) {
    cv::Mat dummy;
    double highThresh = cv::threshold(img, dummy, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
    cv::Canny(img, dummy, 0.5 * highThresh, highThresh);
    return dummy.clone();
}

std::vector<cv::Point2i> gen_points(const cv::Mat& img, unsigned numPts) {
    std::vector<cv::Point2i> points;
    for(int row = 0; row < img.rows; ++row)
        for(int col = 0; col < img.cols; ++col) {
            if(img.at<uchar>(row,col) == 255) {
                points.emplace_back(row, col);
            }
        }

    //std::random_device rd;
    std::shuffle(points.begin(), points.end(), std::mt19937(69));
    return std::vector<cv::Point2i>(points.begin(), points.begin() + numPts);
}

cv::Mat draw_img(const segment_list& segments, const cv::Size size) {
    cv::Mat img = cv::Mat::zeros(size, CV_8UC1);
    for(auto s : segments) {
        cv::line(img, std::get<0>(s), std::get<1>(s), cv::Scalar(255), 1, CV_AA);
        //cv::circle(img, points[i], 1, cv::Scalar(255,255,255), 2);
    }

    return img.clone();
}

long point_dist(const cv::Point2i& a, const cv::Point2i& b) {
    return (a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y);
}

cv::Point2i find_centroid(const std::vector<cv::Point2i>& points) {
    long x = 0; long y = 0;
    for(auto p : points) { x += p.x; y += p.y; }
    if(points.size() > 0) { x /= points.size(); y /= points.size(); }
    std::cout << "X: " << x << ", Y: " << y << std::endl;

    cv::Point2i centroid(x,y);
    cv::Point2i retPt;
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

bool LOP(const cv::Point2i& p1, const cv::Point2i& p2, const cv::Point2i& p3, const cv::Point2i& p4) {
    long dummy = point_dist(p3, p2) * point_dist(p2, p1);
    long sin_a = (p3.x - p2.x) * (p1.y - p2.y) - (p1.x - p2.x) * (p3.y - p2.y);
    sin_a = (dummy != 0) ? (sin_a/dummy) : 1000000;

    long sin_b = (p1.x - p4.x) * (p3.y - p4.y) - (p3.x - p4.x) * (p1.y - p4.y);
    dummy = point_dist(p4, p1) * point_dist(p4, p3);
    sin_b = (dummy != 0) ? (sin_b/dummy) : 1000000;

    long cos_a = (p3.x - p2.x) * (p1.x - p2.x) + (p3.y - p2.y) * (p1.y - p2.y);
    dummy = point_dist(p2, p3) * point_dist(p2, p1);
    cos_a = (dummy != 0) ? (cos_a/dummy) : 1000000;

    long cos_b = (p1.x - p4.x) * (p3.x - p4.x) + (p1.y - p4.y) * (p3.y - p4.y);
    dummy = point_dist(p4, p1) * point_dist(p4, p3);
    cos_b = (dummy != 0) ? (cos_b/dummy) : 1000000;

    return ((cos_a < 0 && cos_b < 0) || (cos_a * sin_b + sin_a * cos_b < 0));
}

namespace tri {
    void basic_alg(cv::Mat& img, std::vector<cv::Point2i>& points) {
        // Fill
    }

    /* Radial sweep */
    segment_list radial(const cv::Mat& img, std::vector<cv::Point2i>& points) {
        segment_list segments;
        std::function<unsigned long(cv::Point2i)> point_hasher = [](cv::Point2i p) -> unsigned long {
            return (53 + std::hash<int>{}(p.x)) * 53 + std::hash<int>{}(p.y);
        };
        std::unordered_map<cv::Point2i, std::vector<cv::Point2i>, decltype(point_hasher)> adj_map(points.size(), point_hasher);

        for(auto p : points) adj_map.emplace(p, std::vector<cv::Point2i>());

        // Find centroidal point (point near centroid)
        auto centroid = find_centroid(points);
        int quadrants[2][2] = {{0, 3}, {1, 2}};

        // Comparator function for clockwise (around centroid) sort
        auto compare_pts = [centroid, &quadrants](cv::Point2i a, cv::Point2i b) {
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

        // Draw lines from centroid to outer points
        std::vector<cv::Point2i>& centroid_adj_list = adj_map[centroid];
        for(auto p : points) {
            segments.emplace_back(centroid, p);
            centroid_adj_list.push_back(p);
            adj_map[p].push_back(centroid);
        }

        std::sort(points.begin(), points.end(), compare_pts);
        // Connect lines in clockwise fashion
        for(std::size_t i = 2; i < points.size(); ++i) {
            segments.emplace_back(points[i-1], points[i]);
            adj_map[points[i-1]].push_back(points[i]);
            adj_map[points[i]].push_back(points[i-1]);
        }

        segments.emplace_back(points.back(), points[1]);
        adj_map[points.back()].push_back(points[1]);
        adj_map[points[1]].push_back(points.back());

        // Make all boundaries convex edges
        for(std::size_t i = 3; i < points.size(); ++i) {
            auto dist1 = point_dist(points[i-2], centroid);
            auto dist2 = point_dist(points[i-1], centroid);
            auto dist3 = point_dist(points[i], centroid);

            if(dist1 > dist2 && dist3 > dist2) {
                segments.emplace_back(points[i-2], points[i]);
                adj_map[points[i-2]].push_back(points[i]);
                adj_map[points[i]].push_back(points[i-2]);
            }
        }

        // LOP - Local Optimization Procedure
        for(auto s : segments) {
            auto p1 = std::get<0>(s);
            auto p2 = std::get<1>(s);

            // Find two other points to complete quadrilateral
            cv::Point2i others[2];
            int num_found = 0;
            auto& vec = adj_map[p2];
            for(auto p : adj_map[p1]) {
                if(num_found == 2) break;
                if(p != p2) {
                    auto result = std::find(vec.begin(), vec.end(), p);
                    if(result != vec.end()) others[num_found++] = *result;
                }
            }

            // Swap according to LOP
            if(num_found == 2 && LOP(p1, others[0], p2, others[1])) {
                auto& v1 = adj_map[p1];
                auto& v2 = adj_map[p2];
                adj_map[p1].erase(std::find(v1.begin(), v1.end(), p2));
                adj_map[p2].erase(std::find(v2.begin(), v2.end(), p1));
                segments.emplace_back(others[0], others[1]);
                adj_map[others[0]].push_back(others[1]);
                adj_map[others[1]].push_back(others[0]);
                segments.erase(std::find(segments.begin(), segments.end(), s));
            }
        }

        return segments;
    }

}

