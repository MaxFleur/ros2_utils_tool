#pragma once

#include "AdvancedInputWidget.hpp"
#include "TF2ToJsonSettings.hpp"
#include "Parameters.hpp"

#include <QPointer>
#include <QWidget>

// Widget used to configure a video encoding out of a ros bag
class TF2ToJsonWidget : public AdvancedInputWidget
{
    Q_OBJECT

public:
    TF2ToJsonWidget(Parameters::TF2ToJsonParameters& parameters,
                    QWidget*                         parent = 0);

private:
    Parameters::TF2ToJsonParameters& m_parameters;

    TF2ToJsonSettings m_settings;
};
