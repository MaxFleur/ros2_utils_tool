#pragma once

#include "BasicInputWidget.hpp"
#include "Parameters.hpp"
#include "SelectableBagTopicSettings.hpp"

#include <QPointer>
#include <QTreeWidgetItem>
#include <QWidget>

class BagTreeWidget;

class QLabel;
class QTreeWidgetItem;
class QVBoxLayout;

// Widget for basic bag tree display, without any additional target file handling
class BasicBagWidget : public BasicInputWidget
{
    Q_OBJECT
public:
    explicit
    BasicBagWidget(Parameters::SelectableBagTopicParameters& parameters,
                   const QString&                            titleText,
                   const QString&                            iconText,
                   const QString&                            settingsText,
                   const QString&                            unselectLabelText,
                   QWidget*                                  parent = 0);

protected slots:
    void
    findSourceButtonPressed();

    void
    itemCheckStateChanged(QTreeWidgetItem* item,
                          int              column);

    virtual void
    handleTreeAfterSource()
    {
    }

    virtual void
    populateTreeWidget() = 0;

    virtual void
    enableOkButton() = 0;

protected:
    QPointer<BagTreeWidget> m_treeWidget;
    QPointer<QLabel> m_unselectLabel;
    QPointer<QVBoxLayout> m_controlsLayout;

    static constexpr int COL_CHECKBOXES = 0;
    static constexpr int COL_TOPIC_NAME = 1;
    static constexpr int COL_TOPIC_TYPE = 2;

private:
    Parameters::SelectableBagTopicParameters& m_parameters;

    SelectableBagTopicSettings m_settings;

    bool m_isPlayBag;
};
