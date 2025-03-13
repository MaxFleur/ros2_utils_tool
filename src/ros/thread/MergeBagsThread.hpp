#pragma once

#include "BasicThread.hpp"
#include "Parameters.hpp"

// Thread used to write an edited ROS bag file
class MergeBagsThread : public BasicThread {
    Q_OBJECT
public:
    explicit
    MergeBagsThread(const Parameters::MergeBagsParameters& parameters,
                    int                                    numberOfThreads,
                    QObject*                               parent = nullptr);

    void
    run() override;

private:
    const Parameters::MergeBagsParameters& m_parameters;

    const int m_numberOfThreads;
};
