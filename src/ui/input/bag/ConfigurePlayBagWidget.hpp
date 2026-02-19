#pragma once

#include "BasicInputWidget.hpp"
#include "PlayBagSettings.hpp"
#include "Parameters.hpp"

#include <QPointer>
#include <QTreeWidgetItem>
#include <QWidget>

class BagTreeWidget;

class QCheckBox;
class QDoubleSpinBox;
class QFormLayout;
class QLabel;

// Widget for setting parameters to playing a bag file
class ConfigurePlayBagWidget : public BasicInputWidget
{
    Q_OBJECT
public:
    explicit
    ConfigurePlayBagWidget(Parameters::PlayBagParameters& parameters,
                           QWidget*                       parent = 0);

private slots:
    void
    findSourceButtonPressed();

    void
    populateWidget();

    void
    itemCheckStateChanged(QTreeWidgetItem* item,
                          int              column);

private:
    QPointer<BagTreeWidget> m_treeWidget;

    QPointer<QLabel> m_unselectLabel;

    QPointer<QFormLayout> m_lowerOptionsLayout;
    QPointer<QCheckBox> m_loopCheckBox;
    QPointer<QDoubleSpinBox> m_rateSpinBox;

    Parameters::PlayBagParameters& m_parameters;

    PlayBagSettings m_settings;

    static constexpr double SPINBOX_LOWER_RANGE = 0.0;
    static constexpr double SPINBOX_UPPER_RANGE = 100.0;

    static constexpr int COL_CHECKBOXES = 0;
    static constexpr int COL_TOPIC_NAME = 1;
    static constexpr int COL_TOPIC_TYPE = 2;
    static constexpr int NUMBER_OF_DECIMALS = 1;
};
