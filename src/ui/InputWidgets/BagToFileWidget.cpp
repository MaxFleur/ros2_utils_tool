#include "BagToFileWidget.hpp"

#include "UtilsROS.hpp"
#include "UtilsUI.hpp"

#include <QCheckBox>
#include <QComboBox>
#include <QFormLayout>
#include <QLabel>
#include <QRadioButton>
#include <QTreeWidget>
#include <QTreeWidgetItem>

BagToFileWidget::BagToFileWidget(Parameters::BagToFileParameters& parameters, QWidget *parent) :
    AdvancedInputWidget(parameters, "Bag To File", ":/icons/bag_to_file", "Bag File:", "File Location:", "bag_to_file", OUTPUT_JSON, parent),
    m_parameters(parameters), m_settings(parameters, "bag_to_file")
{
    m_sourceLineEdit->setToolTip("The source bag file directory.");
    m_targetLineEdit->setToolTip("The target file directory.");

    // Replace the low diskspace widget
    const auto lowDiskspaceItem = m_controlsLayout->takeAt(6);

    m_treeWidget = new QTreeWidget;
    m_treeWidget->setColumnCount(3);
    m_treeWidget->headerItem()->setText(COL_CHECKBOXES, "");
    m_treeWidget->headerItem()->setText(COL_TOPIC_NAME, "Topic Name:");
    m_treeWidget->headerItem()->setText(COL_TOPIC_TYPE, "Topic Type:");
    m_treeWidget->setRootIsDecorated(false);
    m_treeWidget->setVisible(false);

    auto* const formatComboBox = new QComboBox;
    formatComboBox->addItem("json", 0);
    formatComboBox->addItem("yaml", 1);
    formatComboBox->addItem("csv", 2);
    formatComboBox->setToolTip("The file format.");
    formatComboBox->setCurrentText(m_parameters.format);

    auto* const singleFileRadioButton = new QRadioButton("Single File");
    singleFileRadioButton->setToolTip("Export all topics into a single file.");
    singleFileRadioButton->setChecked(m_parameters.singleFile);

    auto* const multipleFilesRadioButton = new QRadioButton("One File per Topic");
    multipleFilesRadioButton->setToolTip("Export each topic into a separate file.");
    multipleFilesRadioButton->setChecked(!m_parameters.singleFile);

    auto* const optionsLayout = new QFormLayout;
    optionsLayout->addRow("File Format:", formatComboBox);
    optionsLayout->addRow("File Structure:", singleFileRadioButton);
    optionsLayout->addRow("", multipleFilesRadioButton);

    m_controlsLayout->addSpacing(5);
    m_controlsLayout->addWidget(m_treeWidget);
    m_controlsLayout->addSpacing(10);
    m_controlsLayout->addLayout(optionsLayout);
    m_controlsLayout->addSpacing(5);
    m_controlsLayout->addWidget(lowDiskspaceItem->widget());
    m_controlsLayout->addStretch();

    // Generally, enable ok only if we have a source and target directory
    enableOkButton(!m_parameters.sourceDirectory.isEmpty() && !m_parameters.targetDirectory.isEmpty());

    connect(m_treeWidget, &QTreeWidget::itemChanged, this, [this] (QTreeWidgetItem* item, int /* column */) {
        // Disable item widgets, this improves distinction between enabed and disabled topics
        m_treeWidget->itemWidget(item, COL_TOPIC_NAME)->setEnabled(item->checkState(COL_CHECKBOXES) == Qt::Checked ? true : false);
        m_treeWidget->itemWidget(item, COL_TOPIC_TYPE)->setEnabled(item->checkState(COL_CHECKBOXES) == Qt::Checked ? true : false);

        const auto rowIndex = m_treeWidget->indexOfTopLevelItem(item);
        writeParameterToSettings(m_parameters.topics[rowIndex].isSelected, item->checkState(COL_CHECKBOXES) == Qt::Checked, m_settings);
    });
    connect(formatComboBox, &QComboBox::currentTextChanged, this, &BagToFileWidget::formatComboBoxTextChanged);
    connect(singleFileRadioButton, &QRadioButton::toggled, this, [this, multipleFilesRadioButton] (bool switched) {
        writeParameterToSettings(m_parameters.singleFile, switched, m_settings);
        multipleFilesRadioButton->setChecked(false);
    });
    connect(multipleFilesRadioButton, &QRadioButton::toggled, this, [this, singleFileRadioButton] (bool switched) {
        writeParameterToSettings(m_parameters.singleFile, !switched, m_settings);
        singleFileRadioButton->setChecked(false);
    });

    setFileFormat(formatComboBox->currentText());
    setTopicTreeWidget();
}


void
BagToFileWidget::findSourceButtonPressed()
{
    AdvancedInputWidget::findSourceButtonPressed();

    m_parameters.topics.clear();
    m_settings.write();
    setTopicTreeWidget();
}


void
BagToFileWidget::setTopicTreeWidget()
{
    if (m_sourceLineEdit->text().isEmpty()) {
        return;
    }

    m_treeWidget->clear();
    m_treeWidget->blockSignals(true);

    const auto& bagMetaData = Utils::ROS::getBagMetadata(m_parameters.sourceDirectory);
    // Fill tree widget with topics
    for (size_t i = 0; i < bagMetaData.topics_with_message_count.size(); i++) {
        const auto topicWithMessageCount = bagMetaData.topics_with_message_count.at(i);
        const auto& topicMetaData = topicWithMessageCount.topic_metadata;

        const auto it = std::find_if(m_parameters.topics.begin(), m_parameters.topics.end(), [topicMetaData] (const auto& bagTopic) {
            return bagTopic.name.toStdString() == topicMetaData.name;
        });
        // If the settings do not contain any topic items, create them
        const auto itemAlreadyExists = it != m_parameters.topics.end();
        if (!itemAlreadyExists) {
            Parameters::BagToFileParameters::BagTopic bagTopic;
            bagTopic.name = QString::fromStdString(topicMetaData.name);
            m_parameters.topics.push_back(bagTopic);
        }

        auto& editBagItem = itemAlreadyExists ? *it : m_parameters.topics.back();

        auto* const item = new QTreeWidgetItem;
        m_treeWidget->addTopLevelItem(item);

        item->setFlags(item->flags() & ~Qt::ItemIsSelectable);
        item->setCheckState(COL_CHECKBOXES, editBagItem.isSelected ? Qt::Checked : Qt::Unchecked);
        // Create item widgets
        auto* const topicNameLabel = new QLabel(QString::fromStdString(topicMetaData.name));
        topicNameLabel->setEnabled(editBagItem.isSelected);

        auto* const topicTypeLabel = new QLabel(QString::fromStdString(topicMetaData.type));
        topicTypeLabel->setEnabled(editBagItem.isSelected);
        auto font = topicTypeLabel->font();
        font.setItalic(true);
        topicTypeLabel->setFont(font);

        m_treeWidget->setItemWidget(item, COL_TOPIC_NAME, topicNameLabel);
        m_treeWidget->setItemWidget(item, COL_TOPIC_TYPE, topicTypeLabel);
    }

    m_treeWidget->resizeColumnToContents(COL_CHECKBOXES);
    m_treeWidget->resizeColumnToContents(COL_TOPIC_NAME);
    m_treeWidget->resizeColumnToContents(COL_TOPIC_TYPE);
    m_treeWidget->blockSignals(false);
    m_treeWidget->setVisible(!m_parameters.allTopics);

    // Also show the checkbox for all topics´
    if (m_allTopicsCheckBox) {
        return;
    }

    m_allTopicsCheckBox = new QCheckBox;
    m_allTopicsCheckBox->setToolTip("Write all bag topics to file.\nIf unchecked, you can specify topics manually.");
    m_allTopicsCheckBox->setCheckState(m_parameters.allTopics ? Qt::Checked : Qt::Unchecked);

    m_basicOptionsFormLayout->addRow("All Topics", m_allTopicsCheckBox);

    connect(m_allTopicsCheckBox, &QCheckBox::stateChanged, this, [this] (int state) {
        m_parameters.allTopics = state == Qt::Checked;
        m_settings.write();
        m_treeWidget->setVisible(state == Qt::Unchecked);
    });
}


void
BagToFileWidget::formatComboBoxTextChanged(const QString& text)
{
    // If the combo box item changes, apply a different appendix to the text in the video line edit
    auto newLineEditText = m_targetLineEdit->text();
    newLineEditText.truncate(newLineEditText.lastIndexOf(QChar('.')));
    newLineEditText += "." + text;
    m_targetLineEdit->setText(newLineEditText);

    writeParameterToSettings(m_parameters.targetDirectory, newLineEditText, m_settings);
    setFileFormat(text);
}
