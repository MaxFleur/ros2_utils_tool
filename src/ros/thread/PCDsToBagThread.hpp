#pragma once

#include "BasicThread.hpp"
#include "Parameters.hpp"

// Thread used to write a set of pcd files to a bag file
class PCDsToBagThread : public BasicThread {
    Q_OBJECT

public:
    explicit
    PCDsToBagThread(const Parameters::PCDsToBagParameters& parameters,
                    unsigned int                           numberOfThreads,
                    QObject*                               parent = nullptr);

    void
    run() override;

private:
    const Parameters::PCDsToBagParameters& m_parameters;

    const unsigned int m_numberOfThreads;
};
