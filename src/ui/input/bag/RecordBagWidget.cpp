#include "RecordBagWidget.hpp"

#include "LowDiskSpaceWidget.hpp"
#include "TopicWidget.hpp"
#include "UtilsUI.hpp"

#include <QCheckBox>
#include <QFormLayout>
#include <QLabel>
#include <QPushButton>
#include <QSet>

RecordBagWidget::RecordBagWidget(Parameters::RecordBagParameters& parameters, QWidget *parent) :
    TopicListingInputWidget(parameters, "Record Bag", ":/icons/tools/record_bag", "record_bag", parent),
    m_parameters(parameters), m_settings(parameters, "record_bag")
{
    m_sourceLineEdit->setText(m_parameters.sourceDirectory);
    m_sourceLineEdit->setToolTip("The target recorded bag file directory.");

    auto* const allTopicsCheckBox = new QCheckBox;
    allTopicsCheckBox->setCheckState(m_parameters.allTopics ? Qt::Checked : Qt::Unchecked);
    allTopicsCheckBox->setToolTip("Record all currently available topics.\nIf unchecked, you can specify topics manually.");

    auto* const basicOptionsFormLayout = new QFormLayout;
    basicOptionsFormLayout->addRow("Bag Location:", m_findSourceLayout);
    basicOptionsFormLayout->addRow("Record all Topics:", allTopicsCheckBox);

    auto* const manageTopicsButtonWidget = new QWidget;
    manageTopicsButtonWidget->setLayout(m_topicButtonLayout);

    m_formLayout = new QFormLayout;
    m_formLayout->addRow("", manageTopicsButtonWidget);

    auto* const topicsWidget = new QWidget;
    topicsWidget->setLayout(m_formLayout);
    topicsWidget->setVisible(!m_parameters.allTopics);

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

    m_controlsLayout->addLayout(basicOptionsFormLayout);
    m_controlsLayout->addSpacing(10);
    m_controlsLayout->addWidget(topicsWidget);
    m_controlsLayout->addSpacing(5);
    m_controlsLayout->addWidget(m_lowDiskSpaceWidget);
    m_controlsLayout->addSpacing(10);
    m_controlsLayout->addWidget(advancedOptionsCheckBox);
    m_controlsLayout->addSpacing(10);
    m_controlsLayout->addWidget(advancedOptionsWidget);
    m_controlsLayout->addStretch();

    const auto addNewTopic = [this] {
        m_parameters.topics.push_back("");
        m_settings.write();
        createNewTopicLineEdit("", m_parameters.topics.size() - 1);
    };
    // Create widgets for already existing topics
    for (auto i = 0; i < m_parameters.topics.size(); i++) {
        createNewTopicLineEdit(m_parameters.topics.at(i), i);
    }
    if (m_parameters.topics.empty()) {
        addNewTopic();
    }

    connect(m_addTopicButton, &QPushButton::clicked, this, [addNewTopic] {
        addNewTopic();
    });
    connect(allTopicsCheckBox, &QCheckBox::stateChanged, this, [this, topicsWidget] (int state) {
        writeParameterToSettings(m_parameters.allTopics, state == Qt::Checked, m_settings);
        topicsWidget->setVisible(state == Qt::Unchecked);
    });
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
    m_okButton->setEnabled(!m_sourceLineEdit->text().isEmpty());
}


void
RecordBagWidget::removeLineEdit(int row)
{
    m_formLayout->removeRow(row);
    m_topicLabels.remove(row);
    m_topicWidgets.remove(row);
    m_parameters.topics.remove(row);
    m_settings.write();
    m_numberOfTopics--;

    for (auto i = 0; i < m_topicLabels.size(); ++i) {
        m_topicLabels[i]->setText("Topic " + QString::number(i + 1) + ":");
    }
}


void
RecordBagWidget::createNewTopicLineEdit(const QString& topicName, int index)
{
    m_topicLabels.push_back(new QLabel("Topic " + QString::number(m_numberOfTopics + 1) + ":"));

    auto* const topicWidget = new TopicWidget(false, m_numberOfTopics != 0, "", topicName);
    m_topicWidgets.push_back(topicWidget);
    // Keep it all inside the main form layout
    // Ensure that the plus button stays below the newly formed widget
    m_formLayout->insertRow(m_formLayout->rowCount() - 1, m_topicLabels.back(), topicWidget);

    connect(topicWidget, &TopicWidget::topicNameChanged, this, [this, index] (const QString& text) {
        writeParameterToSettings(m_parameters.topics[index], text, m_settings);
    });
    connect(topicWidget, &TopicWidget::topicRemoveButtonClicked, this, [this, topicWidget] {
        removeLineEdit(topicWidget->property("id").toInt());
    });

    m_numberOfTopics++;
    for (auto i = 0; i < m_numberOfTopics; i++) {
        m_topicWidgets[i]->setProperty("id", i);
    }
}


std::optional<bool>
RecordBagWidget::areTopicsValid() const
{
    QSet<QString> topicNameSet;
    for (QPointer<QLineEdit> lineEdit : m_topicLineEdits) {
        if (lineEdit->text().isEmpty()) {
            Utils::UI::createCriticalMessageBox("Empty Topic Name!", "Please enter a topic name for every topic!");
            return std::nullopt;
        }

        topicNameSet.insert(lineEdit->text());
    }
    return topicNameSet.size() == m_topicLineEdits.size();
}
