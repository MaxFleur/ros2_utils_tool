#pragma once

#include "BasicThread.hpp"
#include "Parameters.hpp"

// Thread used to write an edited ROS bag file
class EditBagThread : public BasicThread {
    Q_OBJECT
public:
    explicit
    EditBagThread(const Parameters::EditBagParameters& parameters,
                  int                                  numberOfThreads,
                  QObject*                             parent = nullptr);

    void
    run() override;

private:
    const Parameters::EditBagParameters& m_parameters;

    const int m_numberOfThreads;
};
