#pragma once

#include "AdvancedInputWidget.hpp"
#include "BagToFileSettings.hpp"
#include "Parameters.hpp"

#include <QPointer>
#include <QWidget>

class QCheckBox;
class QTreeWidget;

// Widget used to configure writing bag topics to file
class BagToFileWidget : public AdvancedInputWidget
{
    Q_OBJECT

public:
    BagToFileWidget(Parameters::BagToFileParameters& parameters,
                    QWidget*                         parent = 0);

private:
    void
    findSourceButtonPressed() override;

    void
    setTopicTreeWidget();

    void
    formatComboBoxTextChanged(const QString& text);

private:
    QPointer<QCheckBox> m_allTopicsCheckBox;
    QPointer<QTreeWidget> m_treeWidget;

    Parameters::BagToFileParameters& m_parameters;

    BagToFileSettings m_settings;

    static constexpr int COL_CHECKBOXES = 0;
    static constexpr int COL_TOPIC_NAME = 1;
    static constexpr int COL_TOPIC_TYPE = 2;
};
