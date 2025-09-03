#pragma once

#include "AdvancedSettings.hpp"
#include "Parameters.hpp"
#include "TopicComboBoxWidget.hpp"

class QComboBox;
class QLineEdit;

// The widget used to manage writing pcd files out of a ROS bag
class BagToPCDsWidget : public TopicComboBoxWidget
{
    Q_OBJECT

public:
    BagToPCDsWidget(Parameters::AdvancedParameters& parameters,
                    QWidget*                        parent = 0);

private:
    Parameters::AdvancedParameters& m_parameters;

    AdvancedSettings m_settings;
};
