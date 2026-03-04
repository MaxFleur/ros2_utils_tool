#pragma once

#include "BagToImagesSettings.hpp"
#include "Parameters.hpp"
#include "TopicComboBoxWidget.hpp"

class QCheckBox;
class QFormLayout;
class QSlider;

// The widget used to manage writing a ROS bag image message topic to a set of image files
class BagToImagesWidget : public TopicComboBoxWidget
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
