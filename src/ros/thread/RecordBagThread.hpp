#pragma once

#include "BasicThread.hpp"
#include "Parameters.hpp"

// Thread used to record a bag file
class RecordBagThread : public BasicThread {
    Q_OBJECT
public:
    explicit
    RecordBagThread(const Parameters::RecordBagParameters& parameters,
                    QObject*                               parent = nullptr);

    void
    run() override;

private:
    const Parameters::RecordBagParameters& m_parameters;
};
