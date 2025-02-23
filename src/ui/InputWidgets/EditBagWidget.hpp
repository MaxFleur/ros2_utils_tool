#pragma once

#include "BasicBagWidget.hpp"
#include "EditBagSettings.hpp"
#include "UtilsUI.hpp"

#include <QPointer>
#include <QWidget>

class QCheckBox;
class QLabel;

// Widget for editing a bag file
class EditBagWidget : public BasicBagWidget
{
    Q_OBJECT
public:
    explicit
    EditBagWidget(Utils::UI::EditBagParameters& parameters,
                  bool                          checkROS2NameConform,
                  QWidget*                      parent = 0);

private slots:
    void
    createTopicTree(bool newTreeRequested);

    void
    itemCheckStateChanged(QTreeWidgetItem* item,
                          int              column) override;

    void
    okButtonPressed();

private:
    QPointer<QLabel> m_editLabel;
    QPointer<QLabel> m_differentDirsLabel;

    QPointer<QCheckBox> m_deleteSourceCheckBox;
    QPointer<QCheckBox> m_updateTimestampsCheckBox;

    Utils::UI::EditBagParameters& m_parameters;

    EditBagSettings m_settings;

    const bool m_checkROS2NameConform;

    static constexpr int COL_MESSAGE_COUNT = 3;
    static constexpr int COL_RENAMING = 4;

    static constexpr int BUFFER_SPACE = 200;
};
