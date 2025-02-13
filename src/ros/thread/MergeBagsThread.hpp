#pragma once

#include "BasicThread.hpp"
#include "UtilsUI.hpp"

// Thread used to write an edited ROS bag file
class MergeBagsThread : public BasicThread {
    Q_OBJECT
public:
    explicit
    MergeBagsThread(const Utils::UI::MergeBagsInputParameters& parameters,
                    QObject*                                   parent = nullptr);

    void
    run() override;

private:
    const Utils::UI::MergeBagsInputParameters& m_parameters;
};
