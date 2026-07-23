#include "catch_ros2/catch_ros2.hpp"

#include "VideoEncoder.hpp"

#include <opencv2/imgcodecs.hpp>

#include <filesystem>

TEST_CASE("Video Encoder Testing", "[utils]") {
    const auto fourccMP4 = cv::VideoWriter::fourcc('m', 'p', '4', 'v');

    SECTION("Path tests") {
        auto encoder = std::make_unique<VideoEncoder>(fourccMP4);
        REQUIRE(encoder->setVideoWriter("/non/existent/dir/video.mp4", 30, 1280, 720, false, false) == false);
        REQUIRE(encoder->setVideoWriter("./test_video.mp4", 30, 1280, 720, false, false) == true);

        std::filesystem::remove("./test_video.mp4");
    }

    SECTION("Write and verify video test") {
        const std::string videoPath = "./test_video.mp4";

        auto encoder = std::make_unique<VideoEncoder>(fourccMP4);
        encoder->setVideoWriter(videoPath, 30, 1280, 720, false, false);

        cv::Mat frame(720, 1280, CV_8UC3, cv::Scalar(255, 0, 0));
        for (auto i = 0; i < 10; ++i) {
            encoder->writeImageToVideo(frame);
        }
        encoder->release();

        cv::VideoCapture capture(videoPath);
        REQUIRE(capture.isOpened());
        REQUIRE(capture.get(cv::CAP_PROP_FRAME_COUNT) == 10);
        REQUIRE(capture.get(cv::CAP_PROP_FRAME_WIDTH) == 1280);
        REQUIRE(capture.get(cv::CAP_PROP_FRAME_HEIGHT) == 720);
        REQUIRE(capture.get(cv::CAP_PROP_FPS) == 30);

        capture.release();
        std::filesystem::remove(videoPath);
    }

    SECTION("Different codec test") {
        const auto fourccMKV = cv::VideoWriter::fourcc('X', '2', '6', '4');
        auto encoder = std::make_unique<VideoEncoder>(fourccMKV);

        REQUIRE(encoder->setVideoWriter("./test_video.mkv", 30, 320, 240, false, false) == true);

        std::filesystem::remove("./test_video.mkv");
    }
}
