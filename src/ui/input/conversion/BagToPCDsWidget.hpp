#pragma once

#include "AdvancedSettings.hpp"
#include "Parameters.hpp"
#include "TopicComboBoxWidget.hpp"

// The widget used to manage writing a ROS bag point cloud message topic to a set of pcd files
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
