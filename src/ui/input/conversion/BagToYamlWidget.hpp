#pragma once

#include "BagToYamlSettings.hpp"
#include "Parameters.hpp"
#include "TopicComboBoxWidget.hpp"

#include <QPointer>
#include <QWidget>

class BagTreeWidget;

// Widget used to configure writing bag topics to file
class BagToYamlWidget : public TopicComboBoxWidget
{
    Q_OBJECT

public:
    BagToYamlWidget(Parameters::BagToYamlParameters& parameters,
                    QWidget*                         parent = 0);

private:
    Parameters::BagToYamlParameters& m_parameters;

    BagToYamlSettings m_settings;
};
