#pragma once

#include "BasicInputWidget.hpp"
#include "MergeBagsInputSettings.hpp"
#include "UtilsUI.hpp"

#include <QLabel>
#include <QLineEdit>
#include <QPointer>
#include <QTreeWidget>

// Widget for editing a bag file
class MergeBagsWidget : public BasicInputWidget
{
    Q_OBJECT
public:
    explicit
    MergeBagsWidget(Utils::UI::MergeBagsInputParameters& mergeBagParameters,
                    QWidget*                             parent = 0);

private slots:
    void
    setSourceDirectory(bool isFirstSource);

    void
    createTopicTree(bool resetTopicsParameter);

    void
    itemCheckStateChanged(QTreeWidgetItem* item,
                          int              column);

    void
    targetPushButtonPressed();

    void
    okButtonPressed();

private:

    QPointer<QLineEdit> m_secondSourceLineEdit;
    QPointer<QLineEdit> m_targetLineEdit;

    QPointer<QTreeWidget> m_treeWidget;

    QPointer<QLabel> m_sufficientSpaceLabel;

    QPointer<QWidget> m_targetBagNameWidget;

    Utils::UI::MergeBagsInputParameters& m_parameters;

    MergeBagsInputSettings m_settings;

    static constexpr int COL_CHECKBOXES = 0;
    static constexpr int COL_TOPIC_NAME = 1;
    static constexpr int COL_TOPIC_TYPE = 2;
};
