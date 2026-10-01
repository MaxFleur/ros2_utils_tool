#pragma once

#include "BagMessageToFileSettings.hpp"
#include "Parameters.hpp"
#include "TopicComboBoxWidget.hpp"

#include <QPointer>
#include <QWidget>

class BagTreeWidget;

// Widget used to configure writing bag topics to file
class BagMessageToFileWidget : public TopicComboBoxWidget
{
    Q_OBJECT

public:
    BagMessageToFileWidget(Parameters::BagMessageToFileParameters& parameters,
                           QWidget*                                parent = 0);

private:
    Parameters::BagMessageToFileParameters& m_parameters;

    BagMessageToFileSettings m_settings;
};
