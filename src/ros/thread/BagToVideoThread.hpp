#pragma once

#include "BasicThread.hpp"
#include "Parameters.hpp"

// Thread handling encoding a video out of a ROS bag
class BagToVideoThread : public BasicThread {
    Q_OBJECT
public:
    explicit
    BagToVideoThread(const Parameters::BagToVideoParameters& parameters,
                     QObject*                                parent = nullptr);

    void
    run() override;

private:
    const Parameters::BagToVideoParameters& m_parameters;
};
