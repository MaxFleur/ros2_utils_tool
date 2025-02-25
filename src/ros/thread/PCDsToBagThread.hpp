#pragma once

#include "BasicThread.hpp"
#include "Parameters.hpp"

// Thread used to weite a video to a bag
class PCDsToBagThread : public BasicThread {
    Q_OBJECT

public:
    explicit
    PCDsToBagThread(const Parameters::PCDsToBagParameters& parameters,
                    QObject*                               parent = nullptr);

    void
    run() override;

private:
    const Parameters::PCDsToBagParameters& m_parameters;
};
