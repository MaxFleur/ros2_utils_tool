#pragma once

#include "BasicThread.hpp"
#include "UtilsUI.hpp"

// Thread used to write images out of a ROS bag
class WriteToPCDsThread : public BasicThread {
    Q_OBJECT
public:
    explicit
    WriteToPCDsThread(const Utils::UI::AdvancedInputParameters& parameters,
                      QObject*                                  parent = nullptr);

    void
    run() override;

private:
    const Utils::UI::AdvancedInputParameters& m_parameters;
};
