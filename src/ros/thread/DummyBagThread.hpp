#pragma once

#include "BasicThread.hpp"
#include "Parameters.hpp"

// Dummy bag thread, used to write dummy messages to a ROS bag
class DummyBagThread : public BasicThread {
    Q_OBJECT
public:
    explicit
    DummyBagThread(const Parameters::DummyBagParameters& parameters,
                   int                                   numberOfThreads,
                   QObject*                              parent = nullptr);

    void
    run() override;

private:
    const Parameters::DummyBagParameters& m_parameters;

    const int m_numberOfThreads;
};
