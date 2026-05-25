#include "visual_odometry.hpp"
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

    CameraIntrinsics cam;
    if (!cam.loadFromYaml(argv[1]))
    {
        std::cout << "ERROR: Can't load camera parameters." << std::endl;
        return -1;
    }

    VisualOdometry vo(cam);

    auto images = loadImages(argv[2]);

    for (size_t i = 0; i < images.size(); ++i)
    {
        cv::Mat img = cv::imread(images[i], cv::IMREAD_GRAYSCALE);

        if (img.empty())
            continue;

        vo.process(img, i * 0.1);
    }

    return 0;
}