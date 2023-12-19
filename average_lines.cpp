//
// Created by Max Laser on 27.11.23.
//

#include "average_lines.h"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>

cv::Vec4i averageLine(const std::vector<cv::Vec4i>& lines) {
    int x1 = 0, y1 = 0, x2 = 0, y2 = 0;
    for (const auto& line : lines) {
        x1 += line[0];
        y1 += line[1];
        x2 += line[2];
        y2 += line[3];
    }
    int n = lines.size();
    return cv::Vec4i(x1/n, y1/n, x2/n, y2/n);
}

bool areLinesSimilar(const cv::Vec4i& line1, const cv::Vec4i& line2, int max_distance, int max_angle_diff) {
    int dx1 = line1[2] - line1[0];
    int dy1 = line1[3] - line1[1];
    int dx2 = line2[2] - line2[0];
    int dy2 = line2[3] - line2[1];

    double angle1 = std::atan2(dy1, dx1) * 180 / CV_PI;
    double angle2 = std::atan2(dy2, dx2) * 180 / CV_PI;

    double angle_diff = std::abs(angle1 - angle2);
    angle_diff = std::min(angle_diff, 360 - angle_diff);

    double distance = cv::norm(cv::Point(line1[0], line1[1]) - cv::Point(line2[0], line2[1]));

    return distance <= max_distance && angle_diff <= max_angle_diff;
}

std::vector<cv::Vec4i> summarizeLines(std::vector<cv::Vec4i>& lines) {
    std::vector<std::vector<cv::Vec4i>> line_groups;
    for (const auto& line : lines) {
        bool added = false;
        for (auto& group : line_groups) {
            cv::Vec4i avg_line = averageLine(group);
            if (areLinesSimilar(line, avg_line, 40, 30)) {
                group.push_back(line);
                added = true;
                break;
            }
        }
        if (!added) {
            line_groups.push_back({line});
        }
    }

    std::vector<cv::Vec4i> summarized_lines;
    for (const auto& group : line_groups) {
        summarized_lines.push_back(averageLine(group));
    }
    std::cout << "n lines before summarized: " << lines.size() << std::endl;
    std::cout << "n lines after summarized: " << summarized_lines.size() << std::endl;
    return summarized_lines;
}

