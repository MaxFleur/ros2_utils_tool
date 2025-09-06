#pragma once

#include "BasicThread.hpp"
#include "Parameters.hpp"

// Thread used to write an edited ROS bag file
class BagToFileThread : public BasicThread {
    Q_OBJECT
public:
    explicit
    BagToFileThread(const Parameters::BagToFileParameters& parameters,
                    QObject*                               parent = nullptr);

    void
    run() override;

private:
    const Parameters::BagToFileParameters& m_parameters;
};
