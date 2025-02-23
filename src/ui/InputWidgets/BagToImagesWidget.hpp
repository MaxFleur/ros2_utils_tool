#pragma once

#include "AdvancedInputWidget.hpp"
#include "BagToImagesSettings.hpp"
#include "UtilsUI.hpp"

class QCheckBox;
class QFormLayout;
class QLineEdit;
class QSlider;

// The widget used to manage writing images out of a ROS bag
class BagToImagesWidget : public AdvancedInputWidget
{
    Q_OBJECT

public:
    BagToImagesWidget(Utils::UI::BagToImagesParameters& parameters,
                      QWidget*                          parent = 0);

private slots:
    void
    adjustWidgetsToChangedFormat(const QString& text);

private:
    QPointer<QSlider> m_qualitySlider;
    QPointer<QCheckBox> m_optimizeOrBilevelCheckBox;

    QPointer<QFormLayout> m_advancedOptionsFormLayout;

    Utils::UI::BagToImagesParameters& m_parameters;

    BagToImagesSettings m_settings;
};
