#pragma once

#include "BasicThread.hpp"
#include "Parameters.hpp"

// Dummy bag thread, used to create a bag file with dummy data
class DummyBagThread : public BasicThread {
    Q_OBJECT
public:
    explicit
    DummyBagThread(const Parameters::DummyBagParameters& parameters,
                   unsigned int                          numberOfThreads,
                   QObject*                              parent = nullptr);

    void
    run() override;

private:
    const Parameters::DummyBagParameters& m_parameters;

    const unsigned int m_numberOfThreads;
};
