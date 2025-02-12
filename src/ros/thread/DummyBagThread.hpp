#pragma once

#include "BasicThread.hpp"
#include "UtilsUI.hpp"

// Dummy bag thread, used to write dummy messages to a ROS bag
class DummyBagThread : public BasicThread {
    Q_OBJECT
public:
    explicit
    DummyBagThread(const Utils::UI::DummyBagInputParameters& parameters,
                   QObject*                                  parent = nullptr);

    void
    run() override;

private:
    const Utils::UI::DummyBagInputParameters& m_parameters;
};
