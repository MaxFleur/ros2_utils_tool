#pragma once

#include <opencv2/videoio.hpp>

// OpenCV video encoder used to write videos
class VideoEncoder {
public:
    explicit
    VideoEncoder(int fourcc);

    bool
    setVideoWriter(const std::string& directory,
                   int                fps,
                   int                width,
                   int                height,
                   bool               useHardwareAcceleration,
                   bool               useBWImages);

    inline void
    writeImageToVideo(const cv::Mat& mat)
    {
        m_videoWriter.write(mat);
    }

private:
    cv::VideoWriter m_videoWriter;

    int m_fourcc;
};
