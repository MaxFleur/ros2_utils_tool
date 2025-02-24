#pragma once

#include "AdvancedSettings.hpp"
#include "BasicInputWidget.hpp"
#include "Parameters.hpp"

#include <QLineEdit>
#include <QPointer>
#include <QWidget>

class QLabel;
class QTreeWidget;
class QTreeWidgetItem;

// Widget for displaying bag contents for manipulation
class BasicBagWidget : public BasicInputWidget
{
    Q_OBJECT
public:
    explicit
    BasicBagWidget(Parameters::AdvancedParameters& parameters,
                   const QString&                  titleText,
                   const QString&                  iconText,
                   const QString&                  settingsIdentifierText,
                   QWidget*                        parent = 0);

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
                         const QString& secondSourceParameter = QString());

protected:
    QPointer<QTreeWidget> m_treeWidget;

    QPointer<QLineEdit> m_targetLineEdit;

    QPointer<QWidget> m_targetBagNameWidget;

    static constexpr int COL_CHECKBOXES = 0;
    static constexpr int COL_TOPIC_NAME = 1;
    static constexpr int COL_TOPIC_TYPE = 2;

private:
    Parameters::AdvancedParameters& m_parameters;

    AdvancedSettings m_settings;
};
