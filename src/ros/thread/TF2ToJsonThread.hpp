#pragma once

#include "BasicThread.hpp"
#include "Parameters.hpp"

// Dummy bag thread, used to create a bag file with dummy data
class TF2ToJsonThread : public BasicThread {
    Q_OBJECT
public:
    explicit
    TF2ToJsonThread(const Parameters::TF2ToJsonParameters& parameters,
                    QObject*                               parent = nullptr);

    void
    run() override;

private:
    const Parameters::TF2ToJsonParameters& m_parameters;
};
