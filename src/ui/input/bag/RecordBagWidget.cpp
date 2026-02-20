#include "RecordBagWidget.hpp"

#include "BagTreeWidget.hpp"
#include "LowDiskSpaceWidget.hpp"
#include "UtilsROS.hpp"

#include <QCheckBox>
#include <QFormLayout>
#include <QLabel>
#include <QLineEdit>
#include <QHBoxLayout>
#include <QPushButton>
#include <QVBoxLayout>

RecordBagWidget::RecordBagWidget(Parameters::RecordBagParameters& parameters, QWidget *parent) :
    BasicBagWidget(parameters, "Record Bag", ":/icons/tools/record_bag", "record_bag", "Unselect all Topics you don't want to record.", parent),
    m_parameters(parameters), m_settings(parameters, "record_bag")
{
    m_refreshButton = new QPushButton("Refresh List");
    m_refreshButton->setVisible(false);

    auto* const sourceFormLayout = new QFormLayout;
    sourceFormLayout->addRow("Bag Location:", m_findSourceLayout);

    auto* const refreshButtonLayout = new QHBoxLayout;
    refreshButtonLayout->addWidget(m_refreshButton);
    refreshButtonLayout->setAlignment(m_refreshButton, Qt::AlignRight);

    m_lowDiskSpaceWidget = new LowDiskSpaceWidget;

    auto* const advancedOptionsCheckBox = new QCheckBox;
    advancedOptionsCheckBox->setChecked(m_parameters.showAdvancedOptions);
    advancedOptionsCheckBox->setText("Show Advanced Options");

    auto* const includeHiddenTopicsCheckBox = new QCheckBox;
    includeHiddenTopicsCheckBox->setCheckState(m_parameters.includeHiddenTopics ? Qt::Checked : Qt::Unchecked);
    includeHiddenTopicsCheckBox->setToolTip("Whether to include topics not publically shown.");

    auto* const includeUnpublishedTopicsCheckBox = new QCheckBox;
    includeUnpublishedTopicsCheckBox->setCheckState(m_parameters.includeUnpublishedTopics ? Qt::Checked : Qt::Unchecked);
    includeUnpublishedTopicsCheckBox->setToolTip("Whether to include topics where nothing has been published so far.");

    auto* const advancedOptionsFormLayout = new QFormLayout;
    advancedOptionsFormLayout->addRow("Include Hidden Topics:", includeHiddenTopicsCheckBox);
    advancedOptionsFormLayout->addRow("Include Unpublished Topics:", includeUnpublishedTopicsCheckBox);

    auto* const advancedOptionsWidget = new QWidget;
    advancedOptionsWidget->setLayout(advancedOptionsFormLayout);
    advancedOptionsWidget->setVisible(m_parameters.showAdvancedOptions);

    m_controlsLayout->addWidget(m_unselectLabel);
    m_controlsLayout->addWidget(m_treeWidget);
    m_controlsLayout->addLayout(refreshButtonLayout);
    m_controlsLayout->addSpacing(5);
    m_controlsLayout->addLayout(sourceFormLayout);
    m_controlsLayout->addSpacing(5);
    m_controlsLayout->addWidget(m_lowDiskSpaceWidget);
    m_controlsLayout->addSpacing(10);
    m_controlsLayout->addWidget(advancedOptionsCheckBox);
    m_controlsLayout->addSpacing(10);
    m_controlsLayout->addWidget(advancedOptionsWidget);
    m_controlsLayout->addStretch();

    connect(m_refreshButton, &QPushButton::clicked, this, &RecordBagWidget::populateTreeWidget);
    connect(advancedOptionsCheckBox, &QCheckBox::stateChanged, this, [this, advancedOptionsWidget] (int state) {
        writeParameterToSettings(m_parameters.showAdvancedOptions, state == Qt::Checked, m_settings);
        advancedOptionsWidget->setVisible(state == Qt::Checked);
    });
    connect(includeHiddenTopicsCheckBox, &QCheckBox::stateChanged, this, [this] (int state) {
        writeParameterToSettings(m_parameters.includeHiddenTopics, state == Qt::Checked, m_settings);
    });
    connect(includeUnpublishedTopicsCheckBox, &QCheckBox::stateChanged, this, [this] (int state) {
        writeParameterToSettings(m_parameters.includeUnpublishedTopics, state == Qt::Checked, m_settings);
    });

    setPixmapLabelIcon();
    setLowDiskSpaceWidgetVisibility(m_sourceLineEdit->text());
    populateTreeWidget();
}


void
RecordBagWidget::handleTreeAfterSource()
{
    enableOkButton();
}


void
RecordBagWidget::populateTreeWidget()
{
    m_parameters.topics.clear();
    m_treeWidget->clear();
    m_treeWidget->blockSignals(true);

    const auto& currentTopicsAndTypes = Utils::ROS::getTopicInformation();
    for (const auto& topic : currentTopicsAndTypes) {
        // Ignore ROS's own topics
        if (QString::fromStdString(topic.first) == "/parameter_events" || QString::fromStdString(topic.first) == "/rosout" ||
            QString::fromStdString(topic.first) == "/events/read_split") {
            continue;
        }

        m_treeWidget->createItemWithTopicNameAndType(QString::fromStdString(topic.first), QString::fromStdString(topic.second.at(0)), true);
        m_parameters.topics.push_back({ { QString::fromStdString(topic.first) }, true });
    }
    m_settings.write();

    m_treeWidget->resizeColumns();
    // Just take a random item to get its height
    auto* item = m_treeWidget->topLevelItem(m_treeWidget->topLevelItemCount() - 1);
    const auto height = m_treeWidget->visualItemRect(item).height();
    m_treeWidget->setMinimumHeight((height * m_treeWidget->topLevelItemCount()) + HEIGHT_OFFSET);
    m_treeWidget->blockSignals(false);

    m_unselectLabel->setVisible(true);
    m_treeWidget->setVisible(true);
    m_refreshButton->setVisible(true);

    enableOkButton();
}


void
RecordBagWidget::enableOkButton()
{
    const auto isAnyTopicEnabled = std::any_of(m_parameters.topics.begin(), m_parameters.topics.end(), [] (const auto& topic) {
        return topic.isSelected == true;
    });
    m_okButton->setEnabled(isAnyTopicEnabled && !m_sourceLineEdit->text().isEmpty());
}
