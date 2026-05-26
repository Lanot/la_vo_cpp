#pragma once

#include "ivisualizer.hpp"

class OpenCvVisualizer : public IVisualizer
{
private:
    cv::Mat trajectory_;
    cv::Point previous_point_;
    bool starting_point_ = true;

    int trajectory_rows_px_ = 1000;
    int trajectory_cols_px_ = 1800;

    cv::Scalar pose_color_ = cv::Scalar(0, 255, 0); // Green

public:
    OpenCvVisualizer();

    void setPoseColor(const cv::Scalar& pose_color);

    void drawMatches(
        Frame::Ptr prev,
        Frame::Ptr curr,
        const std::vector<cv::DMatch>& matches
    ) override;

    void drawPose(
        const Sophus::SE3d& pose
    ) override;
};
