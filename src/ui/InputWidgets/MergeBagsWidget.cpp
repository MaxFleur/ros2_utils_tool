#include "MergeBagsWidget.hpp"

#include "UtilsROS.hpp"
#include "UtilsUI.hpp"

#include <QDialogButtonBox>
#include <QFileDialog>
#include <QFormLayout>
#include <QLabel>
#include <QMessageBox>
#include <QPushButton>
#include <QShortcut>
#include <QToolButton>
#include <QTreeWidget>
#include <QVBoxLayout>

#include <filesystem>

MergeBagsWidget::MergeBagsWidget(Parameters::MergeBagsParameters& parameters, QWidget *parent) :
    BasicBagWidget(parameters, "Merge Bags", ":/icons/merge_bags", "merge_bags", parent),
    m_parameters(parameters), m_settings(parameters, "merge_bags")
{
    m_secondSourceLineEdit = new QLineEdit;
    auto* const secondSourceButton = new QToolButton;

    auto* const secondSourceLayout = Utils::UI::createLineEditButtonLayout(m_secondSourceLineEdit, secondSourceButton);

    auto* const formLayout = new QFormLayout;
    formLayout->addRow("First Bag Location:", m_findSourceLayout);
    formLayout->addRow("Second Bag Location:", secondSourceLayout);

    if (!std::filesystem::exists(m_parameters.secondSourceDirectory.toStdString()) ||
        !Utils::ROS::doesDirectoryContainBagFile(m_parameters.secondSourceDirectory)) {
        m_parameters.secondSourceDirectory = "";
        writeParameterToSettings(m_parameters.secondSourceDirectory, QString(), m_settings);
    }
    m_secondSourceLineEdit->setText(m_parameters.secondSourceDirectory);

    m_treeWidget->setMinimumHeight(300);

    m_sufficientSpaceLabel = new QLabel("A new bag file will be created, so make sure that enough space is available!");
    m_sufficientSpaceLabel->setVisible(false);

    auto labelFont = m_sufficientSpaceLabel->font();
    labelFont.setBold(true);
    m_sufficientSpaceLabel->setFont(labelFont);

    m_deleteSourceCheckBox->setVisible(false);

    auto* const controlsLayout = new QVBoxLayout;
    controlsLayout->addStretch();
    controlsLayout->addWidget(m_headerPixmapLabel);
    controlsLayout->addWidget(m_headerLabel);
    controlsLayout->addSpacing(30);
    controlsLayout->addLayout(formLayout);
    controlsLayout->addSpacing(10);
    controlsLayout->addWidget(m_treeWidget);
    controlsLayout->addWidget(m_targetBagNameWidget);
    controlsLayout->addLayout(m_diskSpaceLayout);
    controlsLayout->addSpacing(10);
    controlsLayout->addWidget(m_sufficientSpaceLabel);
    controlsLayout->addWidget(m_deleteSourceCheckBox);
    // Give it a more "squishy" look
    controlsLayout->setContentsMargins(30, 30, 30, 30);
    controlsLayout->addStretch();

    auto* const mainLayout = new QVBoxLayout;
    mainLayout->addLayout(controlsLayout);
    mainLayout->addLayout(m_buttonLayout);
    setLayout(mainLayout);

    auto* const okShortCut = new QShortcut(QKeySequence(Qt::Key_Return), this);

    connect(m_findSourceButton, &QPushButton::clicked, this, [this] {
        setSourceDirectory(true);
    });
    connect(secondSourceButton, &QPushButton::clicked, this, [this] {
        setSourceDirectory(false);
    });
    connect(m_dialogButtonBox, &QDialogButtonBox::accepted, this, &MergeBagsWidget::okButtonPressed);
    connect(okShortCut, &QShortcut::activated, this, &MergeBagsWidget::okButtonPressed);

    if (!m_sourceLineEdit->text().isEmpty() && !m_secondSourceLineEdit->text().isEmpty()) {
        createTopicTree(false);
    }
}


void
MergeBagsWidget::setSourceDirectory(bool isFirstSource)
{
    QPointer<QLineEdit> lineEdit = isFirstSource ? m_sourceLineEdit : m_secondSourceLineEdit;
    const auto bagDirectory = QFileDialog::getExistingDirectory(this, "Open Source Bag File", "",
                                                                QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
    if (bagDirectory.isEmpty()) {
        return;
    }
    if (!Utils::ROS::doesDirectoryContainBagFile(bagDirectory)) {
        Utils::UI::createCriticalMessageBox("Invalid bag file!", "The source bag file seems to be invalid or broken!");
        return;
    }
    const auto& otherSourcePath = isFirstSource ? m_parameters.secondSourceDirectory : m_parameters.sourceDirectory;
    if (bagDirectory == otherSourcePath) {
        Utils::UI::createCriticalMessageBox("Equal input bag files!", "The input files are identical. Please select a different file!");
        return;
    }

    lineEdit->setText(bagDirectory);
    writeParameterToSettings(isFirstSource ? m_parameters.sourceDirectory : m_parameters.secondSourceDirectory, bagDirectory, m_settings);
    m_settings.write();
    if (!m_parameters.sourceDirectory.isEmpty() && !m_parameters.secondSourceDirectory.isEmpty()) {
        createTopicTree(true);
    }
}


void
MergeBagsWidget::createTopicTree(bool resetTopicsParameter)
{
    if (resetTopicsParameter) {
        m_parameters.topics.clear();
        m_settings.write();
    }

    m_treeWidget->clear();
    m_treeWidget->blockSignals(true);

    auto topicIndex = 0;
    // Fill tree widget with topics
    const auto fillTreeWithBagTopics = [this, &topicIndex] (const auto& bagFilePath, const auto& topLevelItemText) {
        auto* const topLevelItem = new QTreeWidgetItem;
        topLevelItem->setText(COL_CHECKBOXES, topLevelItemText);

        auto font = topLevelItem->font(COL_CHECKBOXES);
        font.setBold(true);
        topLevelItem->setFont(COL_CHECKBOXES, font);
        m_treeWidget->addTopLevelItem(topLevelItem);

        const auto& bagMetaData = Utils::ROS::getBagMetadata(bagFilePath);

        for (size_t i = 0; i < bagMetaData.topics_with_message_count.size(); i++) {
            const auto topicWithMessageCount = bagMetaData.topics_with_message_count.at(i);
            const auto& topicMetaData = topicWithMessageCount.topic_metadata;

            // Search if the topic already exists in parameters. If it is not there, add it to parameters.
            const auto it = std::find_if(m_parameters.topics.begin(), m_parameters.topics.end(), [topicMetaData, bagFilePath] (const auto& topic) {
                return topic.name.toStdString() == topicMetaData.name && topic.bagDir == bagFilePath;
            });

            const auto itemAlreadyExists = it != m_parameters.topics.end();
            if (!itemAlreadyExists) {
                Parameters::MergeBagsParameters::MergeBagTopic mergeBagTopic;
                mergeBagTopic.name = QString::fromStdString(topicMetaData.name);
                mergeBagTopic.bagDir = bagFilePath;
                m_parameters.topics.push_back(mergeBagTopic);
            }
            auto& mergeBagTopic = itemAlreadyExists ? *it : m_parameters.topics.back();

            auto* const item = new QTreeWidgetItem;
            topLevelItem->addChild(item);

            item->setFlags(item->flags() & ~Qt::ItemIsSelectable);
            item->setCheckState(COL_CHECKBOXES, mergeBagTopic.isSelected ? Qt::Checked : Qt::Unchecked);
            // Create item widgets
            auto* const topicNameLabel = new QLabel(QString::fromStdString(topicMetaData.name));
            topicNameLabel->setEnabled(mergeBagTopic.isSelected);

            auto* const topicTypeLabel = new QLabel(QString::fromStdString(topicMetaData.type));
            topicTypeLabel->setEnabled(mergeBagTopic.isSelected);
            auto font = topicTypeLabel->font();
            font.setItalic(true);
            topicTypeLabel->setFont(font);

            item->setData(COL_TOPIC_NAME, Qt::UserRole, QVariant::fromValue(i + topicIndex));

            m_treeWidget->setItemWidget(item, COL_TOPIC_NAME, topicNameLabel);
            m_treeWidget->setItemWidget(item, COL_TOPIC_TYPE, topicTypeLabel);
        }

        topicIndex += bagMetaData.topics_with_message_count.size();
    };

    fillTreeWithBagTopics(m_parameters.sourceDirectory, "First:");
    m_treeWidget->addTopLevelItem(new QTreeWidgetItem);
    fillTreeWithBagTopics(m_parameters.secondSourceDirectory, "Second:");
    m_treeWidget->expandAll();

    m_treeWidget->resizeColumnToContents(COL_CHECKBOXES);
    m_treeWidget->resizeColumnToContents(COL_TOPIC_NAME);
    m_treeWidget->resizeColumnToContents(COL_TOPIC_TYPE);
    m_treeWidget->setColumnWidth(COL_TOPIC_NAME, m_treeWidget->columnWidth(COL_TOPIC_NAME) + 10);
    // Adjusting the size will for whatever reason reset the column width above
    const auto keptWidth = width();

    m_treeWidget->blockSignals(false);

    m_treeWidget->setVisible(true);
    m_targetBagNameWidget->setVisible(true);
    m_sufficientSpaceLabel->setVisible(true);
    m_deleteSourceCheckBox->setVisible(true);
    m_okButton->setVisible(true);

    adjustSize();
    resize(QSize(keptWidth, height()));

    setDiskSpaceLayoutVisibility();
}


void
MergeBagsWidget::itemCheckStateChanged(QTreeWidgetItem* item, int column)
{
    if (column != COL_CHECKBOXES) {
        return;
    }

    BasicBagWidget::itemCheckStateChanged(item, column);

    // Find corresponding row index in parameters
    auto rowIndex = 0;
    const auto bagDir = item->parent()->text(COL_CHECKBOXES) == "First:" ? m_parameters.sourceDirectory : m_parameters.secondSourceDirectory;
    for (auto i = 0; i < m_parameters.topics.size(); i++) {
        auto* const nameLabel = static_cast<QLabel*>(m_treeWidget->itemWidget(item, COL_TOPIC_NAME));
        // Equal name and bag dir required to find the correct topic
        if (m_parameters.topics.at(i).name == nameLabel->text() && m_parameters.topics.at(i).bagDir == bagDir) {
            rowIndex = i;
            break;
        }
    }
    writeParameterToSettings(m_parameters.topics[rowIndex].isSelected, item->checkState(COL_CHECKBOXES) == Qt::Checked, m_settings);
}


void
MergeBagsWidget::okButtonPressed() const
{
    // Sets remove duplicates, so use such an instance to check for duplicate topic names
    QSet<QString> topicNameSet;
    auto sizeOfSelectedTopics = 0;
    for (const auto& topic : m_parameters.topics) {
        if (!topic.isSelected) {
            continue;
        }

        topicNameSet.insert(topic.name);
        sizeOfSelectedTopics++;
    }

    if (const auto ioParamsValid = areIOParametersValid(sizeOfSelectedTopics, topicNameSet.size(), m_parameters.secondSourceDirectory);
        !ioParamsValid) {
        return;
    }

    emit okPressed();
}
