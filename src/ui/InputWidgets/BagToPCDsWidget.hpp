#pragma once

#include "BasicInputWidget.hpp"
#include "AdvancedInputSettings.hpp"
#include "UtilsUI.hpp"

class QComboBox;
class QLineEdit;

// The widget used to manage writing pcd files out of a ROS bag
class BagToPCDsWidget : public BasicInputWidget
{
    Q_OBJECT

public:
    BagToPCDsWidget(Utils::UI::AdvancedInputParameters& parameters,
                    QWidget*                            parent = 0);

private slots:
    void
    searchButtonPressed();

    void
    pcdsLocationButtonPressed();

    void
    okButtonPressed();

private:
    QPointer<QComboBox> m_topicNameComboBox;
    QPointer<QLineEdit> m_pcdsNameLineEdit;

    Utils::UI::AdvancedInputParameters& m_parameters;

    AdvancedInputSettings m_settings;
};
