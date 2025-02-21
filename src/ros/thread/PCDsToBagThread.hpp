#pragma once

#include "BasicThread.hpp"
#include "UtilsUI.hpp"

// Thread used to weite a video to a bag
class PCDsToBagThread : public BasicThread {
    Q_OBJECT

public:
    explicit
    PCDsToBagThread(const Utils::UI::AdvancedInputParameters& parameters,
                    QObject*                                  parent = nullptr);

    void
    run() override;

private:
    const Utils::UI::AdvancedInputParameters& m_parameters;
};
