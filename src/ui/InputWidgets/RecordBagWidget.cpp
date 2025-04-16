#include "RecordBagWidget.hpp"

#include "UtilsROS.hpp"
#include "UtilsUI.hpp"

#include <QCheckBox>
#include <QCompleter>
#include <QFormLayout>
#include <QLabel>
#include <QPushButton>
#include <QSet>

RecordBagWidget::RecordBagWidget(Parameters::RecordBagParameters& parameters, QWidget *parent) :
    TopicListingInputWidget(parameters, "Record Bag", ":/icons/record_bag", "record_bag", parent),
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

    m_topicsFormLayout = new QFormLayout;
    m_topicsFormLayout->addRow("", manageTopicsButtonWidget);

    auto* const topicsWidget = new QWidget;
    topicsWidget->setLayout(m_topicsFormLayout);
    topicsWidget->setVisible(!m_parameters.allTopics);

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

    connect(m_removeTopicButton, &QPushButton::clicked, this, &RecordBagWidget::removeLineEdit);
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
}


void
RecordBagWidget::removeLineEdit()
{
    m_topicsFormLayout->removeRow(m_parameters.topics.size() - 1);
    m_topicLineEdits.pop_back();
    m_parameters.topics.pop_back();
    m_settings.write();
    m_numberOfTopics--;

    m_removeTopicButton->setEnabled(m_parameters.topics.size() != 1);
}


void
RecordBagWidget::createNewTopicLineEdit(const QString& topicName, int index)
{
    const auto& currentTopicsAndTypes = Utils::ROS::getTopicInformation();
    QStringList wordList;
    for (const auto& currentTopic : currentTopicsAndTypes) {
        wordList.append(QString::fromStdString(currentTopic.first));
    }

    auto* const completer = new QCompleter(wordList);
    completer->setCaseSensitivity(Qt::CaseInsensitive);

    auto* const topicLineEdit = new QLineEdit(topicName);
    topicLineEdit->setPlaceholderText("Enter Topic Name...");
    topicLineEdit->setCompleter(completer);

    connect(topicLineEdit, &QLineEdit::textChanged, this, [this, index] (const QString& text) {
        writeParameterToSettings(m_parameters.topics[index], text, m_settings);
    });

    m_topicsFormLayout->insertRow(m_topicsFormLayout->rowCount() - 1, "Topic " + QString::number(m_numberOfTopics + 1) + ":", topicLineEdit);
    m_topicLineEdits.push_back(topicLineEdit);

    m_numberOfTopics++;
    m_removeTopicButton->setEnabled(m_parameters.topics.size() != 1);
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
