#pragma once

#include "BasicInputWidget.hpp"
#include "DeleteSourceSettings.hpp"
#include "Parameters.hpp"

#include <QPointer>
#include <QWidget>

class QCheckBox;
class QHBoxLayout;
class QLabel;
class QLineEdit;
class QTreeWidget;
class QTreeWidgetItem;

// Widget for displaying bag contents for manipulation
class BasicBagWidget : public BasicInputWidget
{
    Q_OBJECT
public:
    explicit
    BasicBagWidget(Parameters::DeleteSourceParameters& parameters,
                   const QString&                      titleText,
                   const QString&                      iconText,
                   const QString&                      settingsIdentifierText,
                   QWidget*                            parent = 0);

protected slots:
    virtual void
    itemCheckStateChanged(QTreeWidgetItem* item,
                          int              column) = 0;

    void
    targetPushButtonPressed();

protected:
    [[nodiscard]] bool
    areIOParametersValid(int            topicSize,
                         int            topicSizeWithoutDuplicates,
                         const QString& secondSourceParameter = QString()) const;

    void
    setDiskSpaceLayoutVisibility();

protected:
    QPointer<QTreeWidget> m_treeWidget;

    QPointer<QLineEdit> m_targetLineEdit;
    QPointer<QLabel> m_diskSpaceWarningLabel;
    QPointer<QLabel> m_warningIconLabel;
    QPointer<QCheckBox> m_deleteSourceCheckBox;

    QPointer<QWidget> m_targetBagNameWidget;

    QPointer<QHBoxLayout> m_diskSpaceLayout;

    static constexpr int COL_CHECKBOXES = 0;
    static constexpr int COL_TOPIC_NAME = 1;
    static constexpr int COL_TOPIC_TYPE = 2;

private:
    void
    setPixmapLabelIcon() const;

    bool
    event(QEvent *event);

private:
    Parameters::DeleteSourceParameters& m_parameters;

    DeleteSourceSettings m_settings;

    bool m_isDiskSpaceSufficient{ true };

    static constexpr int ICON_SIZE = 25;
};
