#pragma once

#include "AdvancedInputWidget.hpp"
#include "PCDsToBagSettings.hpp"
#include "Parameters.hpp"

#include <QPointer>
#include <QWidget>

// Widget used to write pcd files to a ROS bag file
class PCDsToBagWidget : public AdvancedInputWidget
{
    Q_OBJECT

public:
    PCDsToBagWidget(Parameters::PCDsToBagParameters& parameters,
                    bool                             usePredefinedTopicName,
                    bool                             checkROS2NameConform,
                    QWidget*                         parent = 0);

private slots:
    void
    findSourceButtonPressed() override;

    void
    okButtonPressed() override;

private:
    Parameters::PCDsToBagParameters& m_parameters;

    PCDsToBagSettings m_settings;

    const bool m_checkROS2NameConform;
};
