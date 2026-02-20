#include "ConfigurePlayBagWidget.hpp"

#include "BagTreeWidget.hpp"
#include "UtilsROS.hpp"

#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QVBoxLayout>

ConfigurePlayBagWidget::ConfigurePlayBagWidget(Parameters::PlayBagParameters& parameters, QWidget *parent) :
    BasicBagWidget(parameters, "Play Bag", ":/icons/tools/play_bag", "play_bag", "Unselect all items you don't want to play.", parent),
    m_parameters(parameters), m_settings(parameters, "play_bag")
{
    m_okButton->setVisible(false);

    auto* const sourceFormLayout = new QFormLayout;
    sourceFormLayout->addRow("Source Bag:", m_findSourceLayout);

    m_lowerOptionsLayout = new QFormLayout;
    m_lowerOptionsLayout->setLabelAlignment(Qt::AlignLeft);

    m_rateSpinBox = new QDoubleSpinBox;
    m_rateSpinBox->setDecimals(NUMBER_OF_DECIMALS);
    m_rateSpinBox->setRange(SPINBOX_LOWER_RANGE, SPINBOX_UPPER_RANGE);
    m_rateSpinBox->setValue(m_parameters.rate);

    m_loopCheckBox = new QCheckBox;
    m_loopCheckBox->setTristate(false);
    m_loopCheckBox->setChecked(m_parameters.loop);

    m_controlsLayout->addLayout(sourceFormLayout);
    m_controlsLayout->addSpacing(5);
    m_controlsLayout->addWidget(m_unselectLabel);
    m_controlsLayout->addWidget(m_treeWidget);
    m_controlsLayout->addSpacing(10);
    m_controlsLayout->addLayout(m_lowerOptionsLayout);
    m_controlsLayout->addStretch();

    connect(m_rateSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this] (double value) {
        writeParameterToSettings(m_parameters.rate, value, m_settings);
    });
    connect(m_loopCheckBox, &QCheckBox::stateChanged, this, [this] (int state) {
        writeParameterToSettings(m_parameters.loop, state == Qt::Checked, m_settings);
    });

    if (!m_sourceLineEdit->text().isEmpty()) {
        populateTreeWidget();
    }
}


void
ConfigurePlayBagWidget::handleTreeAfterSource()
{
    m_parameters.topics.clear();
    populateTreeWidget();
}


void
ConfigurePlayBagWidget::populateTreeWidget()
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
            m_parameters.topics.push_back({ { QString::fromStdString(topicMetaData.name) }, true });
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

    enableOkButton();
}


void
ConfigurePlayBagWidget::enableOkButton()
{
    m_okButton->setEnabled(!std::all_of(m_parameters.topics.begin(), m_parameters.topics.end(), [] (const auto& topic) {
        return topic.isSelected == false;
    }));
}
