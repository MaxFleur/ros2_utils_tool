#pragma once

#include "AdvancedInputWidget.hpp"
#include "AdvancedSettings.hpp"
#include "UtilsUI.hpp"

#include <QPointer>
#include <QWidget>

// Widget used to write a video file into a ROSBag
class PCDsToBagWidget : public AdvancedInputWidget
{
    Q_OBJECT

public:
    PCDsToBagWidget(Utils::UI::AdvancedParameters& parameters,
                    bool                           usePredefinedTopicName,
                    bool                           checkROS2NameConform,
                    QWidget*                       parent = 0);

private slots:
    void
    searchButtonPressed() override;

    void
    okButtonPressed() override;

private:
    Utils::UI::AdvancedParameters& m_parameters;

    AdvancedSettings m_settings;

    const bool m_checkROS2NameConform;
};
