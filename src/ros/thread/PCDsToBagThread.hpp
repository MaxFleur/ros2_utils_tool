#pragma once

#include "BasicThread.hpp"
#include "Parameters.hpp"

// Thread used to weite a video to a bag
class PCDsToBagThread : public BasicThread {
    Q_OBJECT

public:
    explicit
    PCDsToBagThread(const Parameters::AdvancedParameters& parameters,
                    QObject*                              parent = nullptr);

    void
    run() override;

private:
    const Parameters::AdvancedParameters& m_parameters;
};
