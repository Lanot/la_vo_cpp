#include "visualizer_opencv.hpp"
#include <iostream>

VisualizerOpenCv::VisualizerOpenCv()
{
    traj_ = cv::Mat::zeros(traj_rows_px_, traj_cols_px_, CV_8UC3);
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
void VisualizerOpenCv::drawPose(
    const Sophus::SE3d& est_pose,
    const Sophus::SE3d& gt_pose
)
{
    Eigen::Vector3d t_est = est_pose.translation();
    Eigen::Vector3d t_gt = gt_pose.translation();

    //std::cout << "POSE: X: " << t.x() << "  Y: " << t.y() << "  Z: " << t.z() << std::endl;

    const int est_x = static_cast<int>(t_est.x() * traj_scale_) + static_cast<int>(traj_cols_px_ / 2);
    const int est_y = static_cast<int>(t_est.z() * traj_scale_) + static_cast<int>(traj_rows_px_ / 2);
    // Usually do NOT use: t.y()

    const int gt_x = static_cast<int>(t_gt.x() * traj_scale_) + static_cast<int>(traj_cols_px_ / 2);
    const int gt_y = static_cast<int>(t_gt.z() * traj_scale_) + static_cast<int>(traj_rows_px_ / 2);
    // Usually do NOT use: t.y()

    const cv::Point current_est_point(est_x, est_y);
    const cv::Point current_gt_point(gt_x, gt_y);

    if (!is_first_point_)
    {
        cv::line(traj_, prev_est_point_, current_est_point, pose_est_color_, 2);
        cv::line(traj_, prev_gt_point_, current_gt_point, pose_gt_color_, 2);
    }

    prev_est_point_ = current_est_point;
    prev_gt_point_ = current_gt_point;
    is_first_point_ = false;

    cv::imshow("Trajectory", traj_);
    cv::waitKey(1);
}
