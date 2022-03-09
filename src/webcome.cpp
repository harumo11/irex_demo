#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

int main(int argc, char const* argv[])
{
    std::cout << "||| Usage" << std::endl;
    std::cout << "||| rosrun irex_demo webcome camera_id" << std::endl;
    std::cout << "||| example : rosrun irex_demo 2" << std::endl;
    std::cout << "||| Hint : camera_id may be between 0 and 5" << std::endl;
    std::cout << std::endl;

    int cam_id = 0;
    if (argc == 1) {
        std::cout << "||| No camera id is specified. Defalut camera id 0 is used" << std::endl;
    } else {
        cam_id = std::stoi(argv[1]);
        std::cout << "||| Camera id " << cam_id << " is specified." << std::endl;
    }

    cv::VideoCapture cap(cam_id);
    if (!cap.isOpened()) {
        std::cout << "||| Error can't open camera" << std::endl;
        std::cout << "||| This program will shutdown" << std::endl;
        std::exit(0);
    }

    cv::Mat src;
    cv::namedWindow("camera", 0);
    while (true) {
        cap >> src;
        cv::imshow("camera", src);
        cv::waitKey(27);
    }
    return 0;
}
