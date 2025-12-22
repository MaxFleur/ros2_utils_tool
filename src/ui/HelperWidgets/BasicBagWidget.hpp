#pragma once

#include "AdvancedInputWidget.hpp"
#include "DeleteSourceSettings.hpp"
#include "Parameters.hpp"

#include <QPointer>
#include <QWidget>

class QCheckBox;
class QHBoxLayout;
class QTreeWidget;
class QTreeWidgetItem;

// Widget for displaying bag contents for manipulation
class BasicBagWidget : public AdvancedInputWidget
{
    Q_OBJECT
public:
    explicit
    BasicBagWidget(Parameters::DeleteSourceParameters& parameters,
                   const QString&                      titleText,
                   const QString&                      iconText,
                   const QString&                      settingsIdentifierText,
                   const int                           outputFormat,
                   QWidget*                            parent = 0);

protected slots:
    virtual void
    itemCheckStateChanged(QTreeWidgetItem* item,
                          int              column) = 0;

protected:
    [[nodiscard]] bool
    areIOParametersValid(int            topicSize,
                         int            topicSizeWithoutDuplicates,
                         const QString& secondSourceParameter = QString()) const;

protected:
    QPointer<QTreeWidget> m_treeWidget;

    QPointer<QCheckBox> m_deleteSourceCheckBox;

    QPointer<QHBoxLayout> m_diskSpaceLayout;

    QPointer<QWidget> m_findTargetWidget;

    static constexpr int COL_CHECKBOXES = 0;
    static constexpr int COL_TOPIC_NAME = 1;
    static constexpr int COL_TOPIC_TYPE = 2;

private:
    Parameters::DeleteSourceParameters& m_parameters;

    DeleteSourceSettings m_settings;

    bool m_isDiskSpaceSufficient{ true };
};
