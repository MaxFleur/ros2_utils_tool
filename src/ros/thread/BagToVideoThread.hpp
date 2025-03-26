#pragma once

#include "BasicThread.hpp"
#include "Parameters.hpp"

// Thread writing a bag images topic to a video file
class BagToVideoThread : public BasicThread {
    Q_OBJECT
public:
    explicit
    BagToVideoThread(const Parameters::BagToVideoParameters& parameters,
                     bool                                    useHardwareAcceleration,
                     QObject*                                parent = nullptr);

    void
    run() override;

private:
    const Parameters::BagToVideoParameters& m_parameters;

    const bool m_useHardwareAcceleration;
};
