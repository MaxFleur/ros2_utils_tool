#include "ConfigurePlayBagWidget.hpp"

#include "BagTreeWidget.hpp"
#include "UtilsROS.hpp"
#include "UtilsUI.hpp"

#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QMessageBox>
#include <QPushButton>
#include <QShortcut>
#include <QVBoxLayout>

#include <filesystem>

ConfigurePlayBagWidget::ConfigurePlayBagWidget(Parameters::PlayBagParameters& parameters, QWidget *parent) :
    BasicInputWidget("Play Bag", ":/icons/tools/play_bag", parent), m_parameters(parameters), m_settings(parameters, "play_bag")
{
    m_sourceLineEdit->setText(m_parameters.sourceDirectory);
    m_okButton->setVisible(false);

    auto* const sourceFormLayout = new QFormLayout;
    sourceFormLayout->addRow("Source Bag:", m_findSourceLayout);

    m_unselectLabel = new QLabel("Unselect all items you don't want to play.");
    m_unselectLabel->setVisible(false);
    auto labelFont = m_unselectLabel->font();
    labelFont.setBold(true);
    m_unselectLabel->setFont(labelFont);

    m_treeWidget = new BagTreeWidget;
    m_treeWidget->setMinimumWidth(350);

    m_lowerOptionsLayout = new QFormLayout;
    m_lowerOptionsLayout->setLabelAlignment(Qt::AlignLeft);

    m_rateSpinBox = new QDoubleSpinBox;
    m_rateSpinBox->setDecimals(NUMBER_OF_DECIMALS);
    m_rateSpinBox->setRange(SPINBOX_LOWER_RANGE, SPINBOX_UPPER_RANGE);
    m_rateSpinBox->setValue(m_parameters.rate);

    m_loopCheckBox = new QCheckBox;
    m_loopCheckBox->setTristate(false);
    m_loopCheckBox->setChecked(m_parameters.loop);

    auto* const controlsLayout = new QVBoxLayout;
    controlsLayout->addStretch();
    controlsLayout->addWidget(m_headerPixmapLabel);
    controlsLayout->addWidget(m_headerLabel);
    controlsLayout->addSpacing(40);
    controlsLayout->addLayout(sourceFormLayout);
    controlsLayout->addSpacing(5);
    controlsLayout->addWidget(m_unselectLabel);
    controlsLayout->addWidget(m_treeWidget);
    controlsLayout->addSpacing(10);
    controlsLayout->addLayout(m_lowerOptionsLayout);
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

    connect(m_findSourceButton, &QPushButton::clicked, this, &ConfigurePlayBagWidget::findSourceButtonPressed);
    connect(m_treeWidget, &QTreeWidget::itemChanged, this, &ConfigurePlayBagWidget::itemCheckStateChanged);
    connect(m_rateSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this] (double value) {
        writeParameterToSettings(m_parameters.rate, value, m_settings);
    });
    connect(m_loopCheckBox, &QCheckBox::stateChanged, this, [this] (int state) {
        writeParameterToSettings(m_parameters.loop, state == Qt::Checked, m_settings);
    });
    connect(m_okButton, &QPushButton::clicked, this, [this] {
        emit okPressed();
    });
    connect(okShortCut, &QShortcut::activated, this, [this] {
        emit okPressed();
    });

    if (!m_sourceLineEdit->text().isEmpty()) {
        populateWidget();
    }
}


void
ConfigurePlayBagWidget::findSourceButtonPressed()
{
    const auto bagDirectory = Utils::UI::isBagDirectoryValid(this);
    if (bagDirectory == std::nullopt) {
        return;
    }

    writeParameterToSettings(m_parameters.sourceDirectory, *bagDirectory, m_settings);
    m_parameters.topics.clear();
    m_settings.write();

    m_sourceLineEdit->setText(*bagDirectory);
    populateWidget();
}


void
ConfigurePlayBagWidget::populateWidget()
{
    m_treeWidget->clear();
    m_treeWidget->blockSignals(true);

    const auto& bagMetaData = Utils::ROS::getBagMetadata(m_parameters.sourceDirectory);
    // Fill tree widget with topics
    for (size_t i = 0; i < bagMetaData.topics_with_message_count.size(); i++) {
        const auto topicWithMessageCount = bagMetaData.topics_with_message_count.at(i);
        const auto& topicMetaData = topicWithMessageCount.topic_metadata;

        const auto it = std::find_if(m_parameters.topics.begin(), m_parameters.topics.end(), [topicMetaData] (const auto& playBagTopic) {
            return playBagTopic.name.toStdString() == topicMetaData.name;
        });

        // If the settings do not contain any topic items, create them
        const auto itemAlreadyExists = it != m_parameters.topics.end();
        if (!itemAlreadyExists) {
            m_parameters.topics.push_back({ QString::fromStdString(topicMetaData.name), QString::fromStdString(topicMetaData.type), true });
        }

        auto& playBagTopic = itemAlreadyExists ? *it : m_parameters.topics.back();
        m_treeWidget->createItemWithTopicNameAndType(QString::fromStdString(topicMetaData.name), QString::fromStdString(topicMetaData.type),
                                                     playBagTopic.isSelected);
    }

    m_treeWidget->resizeColumns();
    m_treeWidget->blockSignals(false);

    m_lowerOptionsLayout->addRow("Rate:", m_rateSpinBox);
    m_lowerOptionsLayout->addRow("Loop File:", m_loopCheckBox);

    m_unselectLabel->setVisible(true);
    m_treeWidget->setVisible(true);
    m_okButton->setVisible(true);

    enableOkButton(!std::all_of(m_parameters.topics.begin(), m_parameters.topics.end(), [] (const auto& topic) {
        return topic.isSelected == false;
    }));
}


void
ConfigurePlayBagWidget::itemCheckStateChanged(QTreeWidgetItem* item, int column)
{
    if (column != COL_CHECKBOXES) {
        return;
    }

    const auto rowIndex = m_treeWidget->indexOfTopLevelItem(item);
    writeParameterToSettings(m_parameters.topics[rowIndex].isSelected, item->checkState(COL_CHECKBOXES) == Qt::Checked, m_settings);

    enableOkButton(!std::all_of(m_parameters.topics.begin(), m_parameters.topics.end(), [] (const auto& topic) {
        return topic.isSelected == false;
    }));
}
