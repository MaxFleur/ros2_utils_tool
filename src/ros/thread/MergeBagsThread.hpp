#pragma once

#include "BasicThread.hpp"
#include "Parameters.hpp"

// Thread used to merge a bag file using two input bag files
// Makes use of the rosbag2_transport API for simplicity and performance
class MergeBagsThread : public BasicThread {
    Q_OBJECT
public:
    explicit
    MergeBagsThread(const Parameters::MergeBagsParameters& parameters,
                    unsigned int                           numberOfThreads,
                    QObject*                               parent = nullptr);

    void
    run() override;

private:
    const Parameters::MergeBagsParameters& m_parameters;

    const unsigned int m_numberOfThreads;
};
