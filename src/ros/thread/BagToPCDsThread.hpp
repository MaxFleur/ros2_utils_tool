#pragma once

#include "BasicThread.hpp"
#include "Parameters.hpp"

// Thread used to write images out of a ROS bag
class BagToPCDsThread : public BasicThread {
    Q_OBJECT
public:
    explicit
    BagToPCDsThread(const Parameters::AdvancedParameters& parameters,
                    QObject*                              parent = nullptr);

    void
    run() override;

private:
    const Parameters::AdvancedParameters& m_parameters;
};
