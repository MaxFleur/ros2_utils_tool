#pragma once

#include "AdvancedBagWidget.hpp"
#include "MergeBagsSettings.hpp"
#include "Parameters.hpp"

#include <QPointer>

class QLabel;
class QLineEdit;

// Widget for merging bag files to a new file
class MergeBagsWidget : public AdvancedBagWidget
{
    Q_OBJECT
public:
    explicit
    MergeBagsWidget(Parameters::MergeBagsParameters& mergeBagParameters,
                    QWidget*                         parent = 0);

private slots:
    void
    findSourceButtonPressed() override;

    void
    createTopicTree(bool resetTopicsParameter);

    void
    itemCheckStateChanged(QTreeWidgetItem* item,
                          int              column) override;

    void
    okButtonPressed() const override;

private:
    QPointer<QLineEdit> m_secondSourceLineEdit;

    QPointer<QLabel> m_sufficientSpaceLabel;

    Parameters::MergeBagsParameters& m_parameters;

    MergeBagsSettings m_settings;

    bool m_secondSourceButtonClicked { false };
};
