#pragma once

#include "BasicBagWidget.hpp"
#include "PlayBagSettings.hpp"
#include "Parameters.hpp"

#include <QPointer>
#include <QWidget>

class QCheckBox;
class QDoubleSpinBox;
class QFormLayout;

// Widget for the parameters for control playing a bag file
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
    QPointer<QDoubleSpinBox> m_offsetSpinBox;

    Parameters::PlayBagParameters& m_parameters;

    PlayBagSettings m_settings;

    bool m_rowsAdded{ false };

    static constexpr double SPINBOX_LOWER_RANGE_RATE = 1.0;
    static constexpr double SPINBOX_LOWER_RANGE_OFFSET = .0;
    static constexpr double SPINBOX_UPPER_RANGE_RATE = 100.0;
    static constexpr double SPINBOX_UPPER_RANGE_OFFSET = 10000.0;

    static constexpr int NUMBER_OF_DECIMALS = 1;
};
