#pragma once

#include "BasicThread.hpp"
#include "Parameters.hpp"

// Thread used to weite a video to a bag
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
