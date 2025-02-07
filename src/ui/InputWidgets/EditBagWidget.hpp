#pragma once

#include "BasicInputWidget.hpp"
#include "EditBagInputSettings.hpp"
#include "UtilsUI.hpp"

#include <QLineEdit>
#include <QPointer>
#include <QWidget>
#include <QTreeWidgetItem>

class QCheckBox;
class QLabel;
class QTreeWidget;

// Widget for editing a bag file
class EditBagWidget : public BasicInputWidget
{
    Q_OBJECT
public:
    explicit
    EditBagWidget(Utils::UI::EditBagInputParameters& parameters,
                  bool                               checkROS2NameConform,
                  QWidget*                           parent = 0);

private slots:
    void
    createTopicTree(bool newTreeRequested);

    void
    itemCheckStateChanged(QTreeWidgetItem* item,
                          int              column);

    void
    targetPushButtonPressed();

    void
    okButtonPressed();

private:
    QPointer<QTreeWidget> m_treeWidget;
    QPointer<QLabel> m_editLabel;
    QPointer<QLabel> m_differentDirsLabel;
    QPointer<QCheckBox> m_deleteSourceCheckBox;
    QPointer<QLineEdit> m_targetLineEdit;
    QPointer<QWidget> m_targetBagNameWidget;

    Utils::UI::EditBagInputParameters& m_parameters;

    EditBagInputSettings m_settings;

    const bool m_checkROS2NameConform;

    static constexpr int COL_CHECKBOXES = 0;
    static constexpr int COL_TOPIC_NAME = 1;
    static constexpr int COL_TOPIC_TYPE = 2;
    static constexpr int COL_MESSAGE_COUNT = 3;
    static constexpr int COL_RENAMING = 4;

    static constexpr int BUFFER_SPACE = 200;
};
