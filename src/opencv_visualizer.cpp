#include "opencv_visualizer.hpp"

#include <iostream>

void OpenCvVisualizer::drawMatches(
    Frame::Ptr prev,
    Frame::Ptr curr,
    const std::vector<cv::DMatch>& matches)
{
    cv::Mat vis;

    std::cout << "Prev -> ID: " << prev->id << std::endl;
    std::cout << "Curr -> ID: " << curr->id << std::endl;

    cv::drawMatches(
        prev->image,
        prev->keypoints,
        curr->image,
        curr->keypoints,
        matches,
        vis
    );

    cv::imshow("matches", vis);

    cv::waitKey(1);
}

void OpenCvVisualizer::drawPose(
    const Sophus::SE3d& pose)
{
    Eigen::Vector3d t = pose.translation();

    std::cout
        << "Pose: "
        << t.transpose()
        << std::endl;
}
