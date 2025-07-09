#pragma once

#include "AdvancedInputWidget.hpp"
#include "BagToImagesSettings.hpp"
#include "Parameters.hpp"

class QCheckBox;
class QFormLayout;
class QSlider;

// The widget used to manage writing images out of a ROS bag
class BagToImagesWidget : public AdvancedInputWidget
{
    Q_OBJECT

public:
    BagToImagesWidget(Parameters::BagToImagesParameters& parameters,
                      QWidget*                           parent = 0);

private slots:
    void
    adjustWidgetsToChangedFormat(const QString& text);

private:
    QPointer<QSlider> m_qualitySlider;
    QPointer<QCheckBox> m_optimizeOrBilevelCheckBox;

    QPointer<QFormLayout> m_advancedOptionsFormLayout;

    Parameters::BagToImagesParameters& m_parameters;

    BagToImagesSettings m_settings;
};
