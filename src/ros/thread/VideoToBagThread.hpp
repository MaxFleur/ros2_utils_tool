#pragma once

#include "BasicThread.hpp"
#include "UtilsUI.hpp"

// Thread used to weite a video to a bag
class VideoToBagThread : public BasicThread {
    Q_OBJECT

public:
    explicit
    VideoToBagThread(const Utils::UI::VideoToBagParameters& parameters,
                     QObject*                               parent = nullptr);

    void
    run() override;

private:
    const Utils::UI::VideoToBagParameters& m_parameters;
};
