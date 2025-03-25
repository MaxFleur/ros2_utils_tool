#pragma once

#include "BasicThread.hpp"
#include "Parameters.hpp"

// Thread used to write a bag images topic to a set of pcd files
class BagToPCDsThread : public BasicThread {
    Q_OBJECT
public:
    explicit
    BagToPCDsThread(const Parameters::AdvancedParameters& parameters,
                    unsigned int                          numberOfThreads,
                    QObject*                              parent = nullptr);

    void
    run() override;

private:
    const Parameters::AdvancedParameters& m_parameters;

    const unsigned int m_numberOfThreads;
};
