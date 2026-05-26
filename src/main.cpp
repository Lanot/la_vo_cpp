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

    ProcessResult res;

    for (size_t i = 0; i < images.size(); ++i)
    {
        cv::Mat img = cv::imread(images[i], cv::IMREAD_GRAYSCALE);

        if (img.empty())
            continue;

        res = vo.process(img, i * 0.1);

        if (!res.matched)
        {
            std::cout << "Tracking failed" << std::endl;
        }
        else if (!res.estimated)
        {
            std::cout << "Pose estimation failed" << std::endl;
        } else
        {
            visualizer->drawMatches(res.prev_frame, res.curr_frame, res.matches);
            visualizer->drawPose(res.global_pose);
        }
    }

    return 0;
}