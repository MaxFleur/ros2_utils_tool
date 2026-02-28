#pragma once

#include "BasicBagWidget.hpp"
#include "PlayBagSettings.hpp"
#include "Parameters.hpp"

#include <QPointer>
#include <QWidget>

class BagTreeWidget;

class QCheckBox;
class QDoubleSpinBox;
class QFormLayout;

// Widget for setting parameters to playing a bag file
class ConfigurePlayBagWidget : public BasicBagWidget
{
    Q_OBJECT
public:
    explicit
    ConfigurePlayBagWidget(Parameters::PlayBagParameters& parameters,
                           QWidget*                       parent = 0);

private slots:
    void
    handleTreeAfterSource() override;

    void
    populateTreeWidget() override;

    void
    enableOkButton() override;

private:
    QPointer<QFormLayout> m_lowerOptionsLayout;
    QPointer<QCheckBox> m_loopCheckBox;
    QPointer<QDoubleSpinBox> m_rateSpinBox;

    Parameters::PlayBagParameters& m_parameters;

    PlayBagSettings m_settings;

    static constexpr double SPINBOX_LOWER_RANGE = 0.0;
    static constexpr double SPINBOX_UPPER_RANGE = 100.0;

    static constexpr int NUMBER_OF_DECIMALS = 1;
};
