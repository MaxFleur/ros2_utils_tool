#pragma once

#include "BasicThread.hpp"
#include "UtilsUI.hpp"

// Thread used to write an edited ROS bag file
class EditBagThread : public BasicThread {
    Q_OBJECT
public:
    explicit
    EditBagThread(const Utils::UI::EditBagParameters& parameters,
                  QObject*                            parent = nullptr);

    void
    run() override;

private:
    const Utils::UI::EditBagParameters& m_parameters;
};
