#pragma once

#include "BasicThread.hpp"
#include "Parameters.hpp"

// Thread used to write a video to a bag file
class VideoToBagThread : public BasicThread {
    Q_OBJECT

public:
    explicit
    VideoToBagThread(const Parameters::VideoToBagParameters& parameters,
                     bool                                    useHardwareAcceleration,
                     QObject*                                parent = nullptr);

    void
    run() override;

private:
    const Parameters::VideoToBagParameters& m_parameters;

    const bool m_useHardwareAcceleration;
};
