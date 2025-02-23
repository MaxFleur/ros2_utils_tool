#pragma once

#include "BasicThread.hpp"
#include "UtilsUI.hpp"

// Thread used to write images out of a ROS bag
class BagToImagesThread : public BasicThread {
    Q_OBJECT
public:
    explicit
    BagToImagesThread(const Utils::UI::BagToImagesParameters& parameters,
                      QObject*                                parent = nullptr);

    void
    run() override;

private:
    const Utils::UI::BagToImagesParameters& m_parameters;
};
