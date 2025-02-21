#pragma once

#include "AdvancedInputWidget.hpp"
#include "AdvancedInputSettings.hpp"
#include "UtilsUI.hpp"

class QComboBox;
class QLineEdit;

// The widget used to manage writing pcd files out of a ROS bag
class BagToPCDsWidget : public AdvancedInputWidget
{
    Q_OBJECT

public:
    BagToPCDsWidget(Utils::UI::AdvancedInputParameters& parameters,
                    QWidget*                            parent = 0);

private:
    Utils::UI::AdvancedInputParameters& m_parameters;

    AdvancedInputSettings m_settings;
};
