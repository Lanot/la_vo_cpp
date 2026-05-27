#include "pose_estimator.hpp"

PoseEstimator::PoseEstimator(const PoseEstimatorConfig& config)
    : config_(config)
{
}

bool PoseEstimator::estimate(
    const std::vector<cv::Point2f>& pts1,
    const std::vector<cv::Point2f>& pts2,
    const cv::Mat& K,
    Sophus::SE3d& relative_pose,
    std::vector<uchar>& status)
{
    if (pts1.size() < 8)
        return false;

    cv::Mat mask;

    cv::Mat E = cv::findEssentialMat(
        pts1,
        pts2,
        K,
        cv::RANSAC,
        0.999,
        1.0,
        mask
    );

    if (E.empty())
        return false;

    cv::Mat R, t;

    int inliers = cv::recoverPose(
        E,
        pts1,
        pts2,
        K,
        R,
        t,
        mask
    );

    if (inliers < 15)
        return false;

    Eigen::Matrix3d rot;
    Eigen::Vector3d trans;

    for (int r = 0; r < 3; ++r)
    {
        for (int c = 0; c < 3; ++c)
        {
            rot(r, c) = R.at<double>(r, c);
        }
    }

    trans <<
        t.at<double>(0),
        t.at<double>(1),
        t.at<double>(2);

    relative_pose = Sophus::SE3d(rot, trans);

    status.assign(mask.begin<uchar>(), mask.end<uchar>());

    return true;
}
