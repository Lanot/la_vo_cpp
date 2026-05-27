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
        std::cout << "Usage: ./la_mono_vo path_to_config/vo.yaml path_to_images" << std::endl;
        return -1;
    }

    std::shared_ptr<OpenCvVisualizer> vo_visualizer = std::make_shared<OpenCvVisualizer>();
    std::shared_ptr<OpenCvVisualizer> gt_visualizer = std::make_shared<OpenCvVisualizer>();

    //vo_visualizer->setPoseColor(cv::Scalar(255, 0, 0));
    gt_visualizer->setPoseColor(cv::Scalar(0, 0, 255));

    Config config(argv[1]);

    VisualOdometry vo(config);

    auto images = loadImagesFromPath(argv[2]);

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
            //gt_visualizer->drawPose(res.curr_frame->pose); // implement it
            vo_visualizer->drawPose(res.curr_frame->pose);
            vo_visualizer->drawMatches(res.prev_frame, res.curr_frame, res.matches);
        }
    }

    return 0;
}
