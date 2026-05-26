#include "opencv_visualizer.hpp"
#include <iostream>

OpenCvVisualizer::OpenCvVisualizer()
{
    trajectory_ =
        cv::Mat::zeros(
            trajectory_rows_px_,
            trajectory_cols_px_,
            CV_8UC3
        );
}

void OpenCvVisualizer::drawMatches(
    Frame::Ptr prev,
    Frame::Ptr curr,
    const std::vector<cv::DMatch>& matches)
{
    cv::Mat vis;

    // std::cout << "Prev -> ID: " << prev->id << std::endl;
    // std::cout << "Curr -> ID: " << curr->id << std::endl;

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
    Eigen::Vector3d t =
        pose.translation();

    std::cout << "POSE: X: " << t.x() << "  Y: " << t.y() << std::endl;

    int x = static_cast<int>(t.x() * 1.5) + static_cast<int>(trajectory_cols_px_ / 2);
    int y = static_cast<int>(t.y() * 1.5) + static_cast<int>(trajectory_rows_px_ / 2);

    cv::Point current_point(x, y);

    if (!first_pose_)
    {
        cv::line(
            trajectory_,
            previous_point_,
            current_point,
            cv::Scalar(0, 255, 0),
            2
        );
    }

    previous_point_ = current_point;
    first_pose_ = false;

    cv::imshow(
        "Trajectory",
        trajectory_
    );

    cv::waitKey(1);
}
