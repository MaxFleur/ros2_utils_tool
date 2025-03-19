#pragma once

#include "BasicInputWidget.hpp"
#include "CompressBagSettings.hpp"
#include "Parameters.hpp"

class QComboBox;
class QLineEdit;

// The widget used to manage compressing a bag file
class CompressBagWidget : public BasicInputWidget
{
    Q_OBJECT

public:
    CompressBagWidget(Parameters::CompressBagParameters& parameters,
                      QWidget*                           parent = 0);

private slots:
    void
    sourceButtonPressed();

    void
    targetButtonPressed();

    void
    okButtonPressed();

private:
    QPointer<QLineEdit> m_targetLineEdit;

    Parameters::CompressBagParameters& m_parameters;

    CompressBagSettings m_settings;
};
