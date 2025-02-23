#pragma once

#include "AdvancedInputWidget.hpp"
#include "AdvancedSettings.hpp"
#include "UtilsUI.hpp"

class QComboBox;
class QLineEdit;

// The widget used to manage writing pcd files out of a ROS bag
class BagToPCDsWidget : public AdvancedInputWidget
{
    Q_OBJECT

public:
    BagToPCDsWidget(Utils::UI::AdvancedParameters& parameters,
                    QWidget*                       parent = 0);

private:
    Utils::UI::AdvancedParameters& m_parameters;

    AdvancedSettings m_settings;
};
