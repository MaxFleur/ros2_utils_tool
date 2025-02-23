#pragma once

#include "BasicBagWidget.hpp"
#include "MergeBagsSettings.hpp"
#include "UtilsUI.hpp"

#include <QLabel>
#include <QLineEdit>
#include <QPointer>

// Widget for editing a bag file
class MergeBagsWidget : public BasicBagWidget
{
    Q_OBJECT
public:
    explicit
    MergeBagsWidget(Utils::UI::MergeBagsParameters& mergeBagParameters,
                    QWidget*                        parent = 0);

private slots:
    void
    setSourceDirectory(bool isFirstSource);

    void
    createTopicTree(bool resetTopicsParameter);

    void
    itemCheckStateChanged(QTreeWidgetItem* item,
                          int              column) override;

    void
    okButtonPressed();

private:
    QPointer<QLineEdit> m_secondSourceLineEdit;

    QPointer<QLabel> m_sufficientSpaceLabel;

    Utils::UI::MergeBagsParameters& m_parameters;

    MergeBagsSettings m_settings;
};
