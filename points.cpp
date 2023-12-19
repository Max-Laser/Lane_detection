//
// Created by Max Laser on 26.11.23.
//

#include "canny.h"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>




cv::Mat summarizePoints(cv::Mat& edges, int threshold = 30) {
    cv::Mat result = edges.clone();
    for (int y = 0; y < result.rows; y++) {
        std::vector<std::pair<int, int>> segments;
        bool inSegment = false;
        for (int x = 0; x < result.cols; x++) {
            if (result.at<uchar>(y, x) == 255) {
                if (!inSegment) {
                    segments.push_back({x, x});
                    inSegment = true;
                } else {
                    segments.back().second = x;
                }
            } else {
                inSegment = false;
            }
        }
        for (size_t i = 0; i < segments.size() - 1; i++) {
            if (segments[i+1].first - segments[i].second <= threshold) {
                int middle = (segments[i].second + segments[i+1].first) / 2;
                for (int x = segments[i].first; x <= segments[i+1].second; x++) {
                    result.at<uchar>(y, x) = 0;
                }
                result.at<uchar>(y, middle) = 255;
            }
        }
    }
    return result;
}

void detectLaneMarkings3(cv::Mat img) {

    double homography_data_psaf1[9] = {
            2.850, 0.835, -340.,
            0, 3.840, -384.,
            0, 0.00425, 1
    };
    // Convert the image to grayscale
    cv::Mat gray, blur, edges;
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
    cv::imshow("gray", gray);
    cv::waitKey(0);
    // Apply Gaussian blur to the image
    cv::GaussianBlur(gray, blur, cv::Size(5, 5), 0, 0);

    // Convert the grayscale image to binary
    cv::Mat img_binary;
    cv::threshold(blur, img_binary, tresh, 255, cv::THRESH_BINARY);//debug images thresh = 100, real image tresh=175
    cv::imshow("img_binary", img_binary);
    cv::waitKey(0);

    //cv::Mat transformed;
    //cv::warpPerspective(img, transformed, homography_data_psaf1, img.size());
    //cv::imshow("transformed", transformed);
    //cv::waitKey(0);


    // Use Canny edge detection to detect edges in the image
    cv::Canny(img_binary, edges, 50, 150);
    cv::imshow("edges", edges);
    cv::waitKey(0);

    cv::Mat summarized = summarizePoints(edges);
    cv::imshow("summarized", summarized);
    cv::waitKey(0);
    // Draw the lines on the original image
    //cv::line(img, cv::Point(left_line[0], left_line[1]), cv::Point(left_line[2], left_line[3]), cv::Scalar(0, 0, 255), 1);
    //cv::line(img, cv::Point(middle_line[0], middle_line[1]), cv::Point(middle_line[2], middle_line[3]), cv::Scalar(0, 255, 0), 2);
    //cv::line(img, cv::Point(right_line[0], right_line[1]), cv::Point(right_line[2], right_line[3]), cv::Scalar(0, 0, 255), 2);

    // Display the image with detected lanes
    cv::imshow("Lane Detection", img);
    cv::waitKey(0);
}