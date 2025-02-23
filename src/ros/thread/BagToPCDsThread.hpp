#pragma once

#include "BasicThread.hpp"
#include "UtilsUI.hpp"

// Thread used to write images out of a ROS bag
class BagToPCDsThread : public BasicThread {
    Q_OBJECT
public:
    explicit
    BagToPCDsThread(const Utils::UI::AdvancedParameters& parameters,
                    QObject*                             parent = nullptr);

    void
    run() override;

private:
    const Utils::UI::AdvancedParameters& m_parameters;
};
