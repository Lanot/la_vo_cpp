#include "opencv_visualizer.hpp"
#include "visual_odometry.hpp"
#include "process_result.hpp"
#include "utils.hpp"

#include <vector>
#include <iostream>

int main(int argc, char** argv)
{
    if (argc < 3)
    {
        std::cout << "Usage: ./la_mono_vo path_to_config/camera.yaml path_to_images" << std::endl;
        return -1;
    }

    std::shared_ptr<IVisualizer> visualizer = std::make_shared<OpenCvVisualizer>();

    CameraIntrinsics camera_intrinsics;
    if (!camera_intrinsics.loadFromYaml(argv[1]))
    {
        std::cout << "ERROR: Can't load camera parameters." << std::endl;
        return -1;
    }

    VisualOdometry vo(camera_intrinsics);

    auto images = loadImages(argv[2]);

    ProcessResult result;

    for (size_t i = 0; i < images.size(); ++i)
    {
        cv::Mat img = cv::imread(images[i], cv::IMREAD_GRAYSCALE);

        if (img.empty())
            continue;

        result = vo.process(img, i * 0.1);

        if (!result.matched)
        {
            std::cout << "Tracking failed" << std::endl;
        }
        else if (!result.estimated)
        {
            std::cout << "Pose estimation failed" << std::endl;
        } else
        {
            visualizer->drawMatches(result.prev_frame, result.curr_frame, result.matches);
            visualizer->drawPose(result.global_pose);
        }
    }

    return 0;
}