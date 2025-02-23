#pragma once

#include "BasicThread.hpp"
#include "UtilsUI.hpp"

// Dummy bag thread, used to write dummy messages to a ROS bag
class DummyBagThread : public BasicThread {
    Q_OBJECT
public:
    explicit
    DummyBagThread(const Utils::UI::DummyBagParameters& parameters,
                   QObject*                             parent = nullptr);

    void
    run() override;

private:
    const Utils::UI::DummyBagParameters& m_parameters;
};
