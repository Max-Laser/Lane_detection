#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include "canny.cpp"
#include "points.cpp"





void detectLaneMarkings(cv::Mat img) {

    // Convert the image to grayscale
    cv::Mat img_gray;
    cv::cvtColor(img, img_gray, cv::COLOR_BGR2GRAY);

    // Convert the grayscale image to binary
    cv::Mat img_binary;
    cv::threshold(img_gray, img_binary, 175, 255, cv::THRESH_BINARY);


    // Display the image
    cv::imshow("Lane Detection", img_binary);
    // cv:    // Display the image with detected lanes
    //    // cv::imshow("Lane Detection", img);
    cv::waitKey(0);
    //
    //    // Use Hough transform to detect lines in the binary image:waitKey(0);
    double pi = CV_PI/180;
    std::cout << "CV_PI/180 = " << pi << std::endl;
    // Use Hough transform to detect lines in the binary image
    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(img_binary, lines, 1, CV_PI/180, 20, 5, 10);
    std::cout << "lines.size() = " << lines.size() << std::endl;
    // Draw the lines
    for( size_t i = 0; i < lines.size(); i++ )
    {
        cv::Vec4i l = lines[i];
        cv::line( img, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(100,100,0), 3, cv::LINE_AA);
        if(i%10==0){//
            //cv::imshow("Lane Detection", img);
            //cv::waitKey(0);
        }
    }
    cv::imshow("Lane Detection", img);
    cv::waitKey(0);

    // Separate lines into left, middle, and right based on their x-position
    std::vector<cv::Vec4i> left_lines, middle_lines, right_lines;
    for (cv::Vec4i line : lines) {
        int x1 = line[0], x2 = line[2];
        if (x1 < img.cols / 3 && x2 < img.cols / 3) {
            left_lines.push_back(line);
        } else if (x1 > 2 * img.cols / 3 && x2 > 2 * img.cols / 3) {
            right_lines.push_back(line);
        } else {
            middle_lines.push_back(line);
        }
    }

    for( size_t i = 0; i < left_lines.size(); i++ )
    {

        cv::Vec4i l = left_lines[i];
        cv::line( img, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0,100,100), 3, cv::LINE_AA);
    }

    for( size_t i = 0; i < middle_lines.size(); i++ )
    {
        cv::Vec4i l = middle_lines[i];
        cv::line( img, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(50,50,0), 3, cv::LINE_AA);
    }

    for( size_t i = 0; i < right_lines.size(); i++ )
    {
        cv::Vec4i l = right_lines[i];
        cv::line( img, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(50,0,50), 3, cv::LINE_AA);
    }

    // Average the position of each of the lines
    cv::Vec4i left_line = cv::mean(left_lines);
    cv::Vec4i middle_line = cv::mean(middle_lines);
    cv::Vec4i right_line = cv::mean(right_lines);

    // Draw the lines on the original image
    cv::line(img, cv::Point(left_line[0], left_line[1]), cv::Point(left_line[2], left_line[3]), cv::Scalar(0, 0, 255), 1);
    cv::line(img, cv::Point(middle_line[0], middle_line[1]), cv::Point(middle_line[2], middle_line[3]), cv::Scalar(0, 255, 0), 2);
    cv::line(img, cv::Point(right_line[0], right_line[1]), cv::Point(right_line[2], right_line[3]), cv::Scalar(0, 0, 255), 2);

    // Display the image with detected lanes
    cv::imshow("Lane Detection", img);
    cv::waitKey(0);
}




int main() {
    // Load the image
    //cv::Mat img = cv::imread("/Users/maxlaser/Desktop/02_AF/code/test/img.png");
    //cv::Mat img = cv::imread("/Users/maxlaser/Desktop/02_AF/code/test/img_curve.png");
    //cv::Mat img = cv::imread("/Users/maxlaser/Desktop/02_AF/code/af_geisterfahrer/ws_geisterfahrer/src/psaf_lane_detection/test/resources/images/psaf1_circle_outer/00003.png");

    cv::Mat img = cv::imread("/Users/maxlaser/Desktop/02_AF/code/test/binarytrack3.png");
    // Check if the image was loaded successfully
    if (img.empty()) {
        std::cout << "Failed to load image" << std::endl;
        return -1;
    }

    // Call the detectLaneMarkings function
    detectLaneMarkings2(img);
    //detectLaneMarkings3(img);
    // detectLaneMarkings(img);

    return 0;
}