#pragma once

#include "BasicThread.hpp"
#include "Parameters.hpp"

// Thread used to write a compressed to an uncompressed file (or vice versa)
// Makes use of the rosbag2_transport API for simplicity and performance
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
