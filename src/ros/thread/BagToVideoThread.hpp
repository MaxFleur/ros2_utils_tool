#pragma once

#include "BasicThread.hpp"
#include "UtilsUI.hpp"

// Thread handling encoding a video out of a ROS bag
class BagToVideoThread : public BasicThread {
    Q_OBJECT
public:
    explicit
    BagToVideoThread(const Utils::UI::BagToVideoParameters& parameters,
                     QObject*                               parent = nullptr);

    void
    run() override;

private:
    const Utils::UI::BagToVideoParameters& m_parameters;
};
