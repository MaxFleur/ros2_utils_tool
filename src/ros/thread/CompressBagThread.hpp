#pragma once

#include "BasicThread.hpp"
#include "Parameters.hpp"

// Thread used to compress a bag file
class CompressBagThread : public BasicThread {
    Q_OBJECT
public:
    explicit
    CompressBagThread(const Parameters::CompressBagParameters& parameters,
                      int                                      numberOfThreads,
                      QObject*                                 parent = nullptr);

    void
    run() override;

private:
    const Parameters::CompressBagParameters& m_parameters;

    int m_numberOfThreads;
};
