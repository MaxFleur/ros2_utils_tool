#include "RecordBagWidget.hpp"

#include "BagTreeWidget.hpp"
#include "LowDiskSpaceWidget.hpp"
#include "UtilsROS.hpp"
#include "UtilsUI.hpp"

#include <QCheckBox>
#include <QFileDialog>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QShortcut>
#include <QVBoxLayout>

RecordBagWidget::RecordBagWidget(Parameters::RecordBagParameters& parameters, QWidget *parent) :
    BasicInputWidget("Record Bag", ":/icons/tools/record_bag", parent),
    m_parameters(parameters), m_settings(parameters, "record_bag")
{
    m_sourceLineEdit->setText(m_parameters.sourceDirectory);
    m_sourceLineEdit->setToolTip("The target recorded bag file directory.");

    m_unselectTopicsLabel = new QLabel("Unselect all Topics you don't want to record.");
    m_unselectTopicsLabel->setVisible(false);
    auto font = m_unselectTopicsLabel->font();
    font.setBold(true);
    m_unselectTopicsLabel->setFont(font);

    m_treeWidget = new BagTreeWidget;
    m_treeWidget->setMinimumWidth(380);

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

    auto* const controlsLayout = new QVBoxLayout;
    controlsLayout->addStretch();
    controlsLayout->addSpacing(10);
    controlsLayout->addWidget(m_headerPixmapLabel);
    controlsLayout->addWidget(m_headerLabel);
    controlsLayout->addSpacing(40);
    controlsLayout->addWidget(m_unselectTopicsLabel);
    controlsLayout->addWidget(m_treeWidget);
    controlsLayout->addLayout(refreshButtonLayout);
    controlsLayout->addSpacing(5);
    controlsLayout->addLayout(sourceFormLayout);
    controlsLayout->addSpacing(5);
    controlsLayout->addWidget(m_lowDiskSpaceWidget);
    controlsLayout->addSpacing(10);
    controlsLayout->addWidget(advancedOptionsCheckBox);
    controlsLayout->addSpacing(10);
    controlsLayout->addWidget(advancedOptionsWidget);
    controlsLayout->addStretch();

    auto* const controlsSqueezedLayout = new QHBoxLayout;
    controlsSqueezedLayout->addStretch();
    controlsSqueezedLayout->addLayout(controlsLayout);
    controlsSqueezedLayout->addStretch();

    auto* const mainLayout = new QVBoxLayout;
    mainLayout->addLayout(controlsSqueezedLayout);
    mainLayout->addLayout(m_buttonLayout);
    setLayout(mainLayout);

    auto* const okShortCut = new QShortcut(QKeySequence(Qt::Key_Return), this);

    connect(m_treeWidget, &QTreeWidget::itemChanged, this, &RecordBagWidget::itemCheckStateChanged);
    connect(m_refreshButton, &QPushButton::clicked, this, &RecordBagWidget::populateTreeWidget);
    connect(m_findSourceButton, &QPushButton::clicked, this, &RecordBagWidget::findSourceButtonPressed);
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
    connect(m_okButton, &QPushButton::clicked, this, [this] {
        emit okPressed();
    });
    connect(okShortCut, &QShortcut::activated, this, [this] {
        emit okPressed();
    });

    setPixmapLabelIcon();
    setLowDiskSpaceWidgetVisibility(m_sourceLineEdit->text());
    populateTreeWidget();
}


void
RecordBagWidget::findSourceButtonPressed()
{
    const auto fileName = QFileDialog::getSaveFileName(this, "Save Bag File");
    if (fileName.isEmpty()) {
        return;
    }

    writeParameterToSettings(m_parameters.sourceDirectory, fileName, m_settings);
    m_settings.write();
    m_sourceLineEdit->setText(fileName);
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
        m_parameters.topics.push_back({ QString::fromStdString(topic.first), true });
    }
    m_settings.write();

    m_treeWidget->resizeColumns();
    // Just take a random item to get its height
    auto* item = m_treeWidget->topLevelItem(m_treeWidget->topLevelItemCount() - 1);
    const auto height = m_treeWidget->visualItemRect(item).height();
    m_treeWidget->setMinimumHeight((height * m_treeWidget->topLevelItemCount()) + HEIGHT_OFFSET);
    m_treeWidget->blockSignals(false);

    m_unselectTopicsLabel->setVisible(true);
    m_treeWidget->setVisible(true);
    m_refreshButton->setVisible(true);

    enableOkButton(!m_sourceLineEdit->text().isEmpty());
}


void
RecordBagWidget::itemCheckStateChanged(QTreeWidgetItem* item, int column)
{
    if (column != COL_CHECKBOXES) {
        return;
    }

    const auto rowIndex = m_treeWidget->indexOfTopLevelItem(item);
    writeParameterToSettings(m_parameters.topics[rowIndex].isSelected, item->checkState(COL_CHECKBOXES) == Qt::Checked, m_settings);

    const auto isAnyTopicEnabled = std::any_of(m_parameters.topics.begin(), m_parameters.topics.end(), [] (const auto& topic) {
        return topic.isSelected == true;
    });
    enableOkButton(isAnyTopicEnabled && !m_sourceLineEdit->text().isEmpty());
}
