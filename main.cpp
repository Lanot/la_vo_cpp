#include "visualizer_opencv.hpp"
#include "visual_odometry.hpp"
#include "visual_odometry_result.hpp"
#include "feature_tracker_factory.hpp"
#include "utils.hpp"

#include <vector>
#include <iostream>

int main(int argc, char** argv)
{
    if (argc < 4)
    {
        std::cout << "Usage: ./la_mono_vo path_to_config/vo.yaml path_to_images path_to_ground_truth_file" << std::endl;
        return -1;
    }

    std::shared_ptr<VisualizerOpenCv> vo_visualizer = std::make_shared<VisualizerOpenCv>();
    std::shared_ptr<VisualizerOpenCv> gt_visualizer = std::make_shared<VisualizerOpenCv>();

    vo_visualizer->setPoseColor(cv::Scalar(255, 0, 0));
    gt_visualizer->setPoseColor(cv::Scalar(0, 0, 255));

    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();

    Config config(argv[1]);
    config.validate();

    auto tracker = FeatureTrackerFactory::create(config.trackerConfig());
    auto estimator = std::make_shared<PoseEstimator>(config.poseEstimatorConfig());

    VisualOdometry vo(config, tracker, estimator);

    std::vector<std::string> images = loadImagesFromPath(argv[2]);
    std::vector<std::string> gt_poses = loadPosesFromPath(argv[3]);

    for (size_t i = 0; i < images.size(); ++i)
    {
        cv::Mat img = cv::imread(images[i], cv::IMREAD_GRAYSCALE);

        if (config.optimizationConfig().use_clahe)
        {
            clahe->apply(img, img);
        }

        if (img.empty())
            continue;

        VisualOdometryResult res = vo.process(img, i * 0.1);

        if (!res.matched)
        {
            std::cout << "Tracking failed" << std::endl;
        }
        else if (!res.estimated)
        {
            std::cout << "Pose estimation failed" << std::endl;
        }
        else
        {
            std::cout << "Pose estimation succeeded: #" << i << std::endl;

            // Draw VO Estimation Pose and Matches
            vo_visualizer->drawPose(res.curr_frame->pose);
            vo_visualizer->drawMatches(res.prev_frame, res.curr_frame, res.matches);

            Eigen::Vector3d vo_t = res.curr_frame->pose.translation();
            std::cout << "VO POSE: X: " << vo_t.x() << "  Y: " << vo_t.y() << std::endl;

            // Draw Kitti Ground Truth Pose
            Sophus::SE3d tg_pose = kittiLinePoseToSophusPose(gt_poses[i]);
            gt_visualizer->drawPose(tg_pose);

            Eigen::Vector3d gt_t = tg_pose.translation();
            std::cout << "GT POSE: X: " << gt_t.x() << "  Y: " << gt_t.y() << std::endl;
        }
    }

    return 0;
}
