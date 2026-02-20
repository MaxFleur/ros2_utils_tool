#pragma once

#include "BasicInputWidget.hpp"
#include "Parameters.hpp"
#include "RecordBagSettings.hpp"

#include <QPointer>
#include <QTreeWidgetItem>
#include <QWidget>

class BagTreeWidget;

class QLabel;
class QPushButton;

// Widget used to manage recording a bag file
class RecordBagWidget : public BasicInputWidget
{
    Q_OBJECT

public:
    RecordBagWidget(Parameters::RecordBagParameters& parameters,
                    QWidget*                         parent = 0);

private slots:
    void
    findSourceButtonPressed();

    void
    populateTreeWidget();

    void
    itemCheckStateChanged(QTreeWidgetItem* item,
                          int              column);

private:
    QPointer<BagTreeWidget> m_treeWidget;
    QPointer<QPushButton> m_refreshButton;
    QPointer<QLabel> m_unselectTopicsLabel;

    Parameters::RecordBagParameters& m_parameters;

    RecordBagSettings m_settings;

    static constexpr int COL_CHECKBOXES = 0;
    static constexpr int COL_TOPIC_NAME = 1;
    static constexpr int COL_TOPIC_TYPE = 2;

    static constexpr int HEIGHT_OFFSET = 80;
};
