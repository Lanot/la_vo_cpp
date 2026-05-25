#include "visual_odometry.hpp"

#include <vector>
#include <iostream>

VisualOdometry::VisualOdometry(const CameraIntrinsics& camera_intrinsics)
    : camera_intrinsics_(camera_intrinsics)
{
    global_pose_ = Sophus::SE3d();
}

bool VisualOdometry::process(
    const cv::Mat& image,
    double timestamp)
{
    current_ = Frame::create();

    current_->image = image.clone();
    current_->timestamp = timestamp;

    tracker_.extract(current_);

    if (!previous_)
    {
        previous_ = current_;
        return true;
    }

    std::vector<cv::DMatch> matches;

    std::vector<cv::Point2f> pts1, pts2;

    bool matched = tracker_.match(
        previous_,
        current_,
        matches,
        pts1,
        pts2
    );

    if (!matched)
    {
        std::cout << "Tracking failed" << std::endl;
        previous_ = current_;
        return false;
    }

    Sophus::SE3d relative_pose;

    std::vector<uchar> status;

    bool estimated = estimator_.estimate(
        pts1,
        pts2,
        camera_intrinsics_.K(),
        relative_pose,
        status
    );

    if (!estimated)
    {
        std::cout << "Pose estimation failed" << std::endl;
        previous_ = current_;
        return false;
    }

    global_pose_ = global_pose_ * relative_pose;

    current_->pose = global_pose_;

    // ----------------------------------------------------- //
    cv::Mat vis;

    cv::drawMatches(
        previous_->image,
        previous_->keypoints,
        current_->image,
        current_->keypoints,
        matches,
        vis
    );

    cv::imshow("matches", vis);
    cv::waitKey(1);

    Eigen::Vector3d t = global_pose_.translation();

    std::cout
        << "Pose: "
        << t.transpose()
        << std::endl;
    // ----------------------------------------------------- //

    previous_ = current_;

    return true;
}

Sophus::SE3d VisualOdometry::currentPose() const
{
    return global_pose_;
}
