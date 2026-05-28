#include "visualizer_opencv.hpp"
#include <iostream>

VisualizerOpenCv::VisualizerOpenCv()
{
    traj_ =
        cv::Mat::zeros(
            traj_rows_px_,
            traj_cols_px_,
            CV_8UC3
        );
}

void VisualizerOpenCv::setPoseColor(const cv::Scalar& pose_color)
{
    pose_color_ = pose_color;
}

void VisualizerOpenCv::drawMatches(
    Frame::Ptr prev,
    Frame::Ptr curr,
    const std::vector<cv::DMatch>& matches
)
{
    if (matches.empty())
        return;

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

// Typical KITTI visualization
//     z (forward)
//     ^
//     |
//     ------> x (right)
void VisualizerOpenCv::drawPose(const Sophus::SE3d& pose)
{
    Eigen::Vector3d t =
        pose.translation();

    //std::cout << "POSE: X: " << t.x() << "  Y: " << t.y() << "  Z: " << t.z() << std::endl;

    int dpt_x = static_cast<int>(t.x() * traj_scale_) + static_cast<int>(traj_cols_px_ / 2);
    int dpt_y = static_cast<int>(t.z() * traj_scale_) + static_cast<int>(traj_rows_px_ / 2); // Usually do NOT use: t.y()


    cv::Point current_point(dpt_x, dpt_y);

    if (!is_first_point_)
    {
        cv::line(
            traj_,
            prev_point_,
            current_point,
            pose_color_,
            2
        );
    }

    prev_point_ = current_point;
    is_first_point_ = false;

    cv::imshow(
        "Trajectory",
        traj_
    );

    cv::waitKey(1);
}
