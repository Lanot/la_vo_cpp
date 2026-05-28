#pragma once

#include "ivisualizer.hpp"

class VisualizerOpenCv : public IVisualizer
{
protected:
    cv::Mat traj_;
    cv::Point prev_point_;
    bool is_first_point_ = true;

    int traj_rows_px_ = 1400;
    int traj_cols_px_ = 1400;
    double traj_scale_ = 1.0;

    cv::Scalar pose_color_ = cv::Scalar(0, 255, 0); // Green

public:
    VisualizerOpenCv();

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
