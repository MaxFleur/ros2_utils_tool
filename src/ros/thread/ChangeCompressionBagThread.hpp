#pragma once

#include "BasicThread.hpp"
#include "Parameters.hpp"

// Thread used to compress a bag file
class ChangeCompressionBagThread : public BasicThread {
    Q_OBJECT
public:
    explicit
    ChangeCompressionBagThread(const Parameters::CompressBagParameters& parameters,
                               int                                      numberOfThreads,
                               bool                                     compress,
                               QObject*                                 parent = nullptr);

    void
    run() override;

private:
    const Parameters::CompressBagParameters& m_parameters;

    int m_numberOfThreads;
    bool m_compress;
};
