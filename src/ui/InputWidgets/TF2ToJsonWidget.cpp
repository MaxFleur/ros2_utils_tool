#include "TF2ToJsonWidget.hpp"

#include "UtilsUI.hpp"

#include <QCheckBox>
#include <QFormLayout>
#include <QLabel>
#include <QRadioButton>

TF2ToJsonWidget::TF2ToJsonWidget(Parameters::TF2ToJsonParameters& parameters, QWidget *parent) :
    AdvancedInputWidget(parameters, "TF2 to Json", ":/icons/tf2_to_json", "Bag File:", "Json Location:", "tf2_to_json", OUTPUT_JSON, parent),
    m_parameters(parameters), m_settings(parameters, "tf2_to_json")
{
    m_sourceLineEdit->setToolTip("The source bag file directory.");
    m_topicNameComboBox->setToolTip("The transformation messages topic.\nIf the bag contains multiple transformation topics, you can choose one of them.");
    m_targetLineEdit->setToolTip("The target json file directory.");

    m_basicOptionsFormLayout->insertRow(1, "Topic Name:", m_topicNameComboBox);

    auto* const advancedOptionsCheckBox = new QCheckBox;
    advancedOptionsCheckBox->setChecked(m_parameters.showAdvancedOptions);
    advancedOptionsCheckBox->setText("Show Advanced Options");

    auto* const compactRadioButton = new QRadioButton("Compact");
    compactRadioButton->setToolTip("Use a compact json format to keep the file as short as possible.");
    compactRadioButton->setChecked(m_parameters.compactOutput);

    auto* const indentedRadioButton = new QRadioButton("Indented");
    indentedRadioButton->setToolTip("Indent the json file for better readability.");
    indentedRadioButton->setChecked(!m_parameters.compactOutput);

    auto* const keepTimestampsCheckBox = Utils::UI::createCheckBox("Keep the message's timestamp into the file.", m_parameters.keepTimestamps);

    auto* const advancedOptionsFormLayout = new QFormLayout;
    advancedOptionsFormLayout->addRow("Formatting:", compactRadioButton);
    advancedOptionsFormLayout->addRow("", indentedRadioButton);
    advancedOptionsFormLayout->addRow("Save Message Timestamps:", keepTimestampsCheckBox);

    auto* const advancedOptionsWidget = new QWidget;
    advancedOptionsWidget->setLayout(advancedOptionsFormLayout);
    advancedOptionsWidget->setVisible(m_parameters.showAdvancedOptions);

    m_controlsLayout->addWidget(advancedOptionsCheckBox);
    m_controlsLayout->addSpacing(10);
    m_controlsLayout->addWidget(advancedOptionsWidget);
    m_controlsLayout->addStretch();

    // Generally, enable ok only if we have a source and target directory and a topic name
    enableOkButton(!m_parameters.sourceDirectory.isEmpty() &&
                   !m_parameters.topicName.isEmpty() && !m_parameters.targetDirectory.isEmpty());

    connect(advancedOptionsCheckBox, &QCheckBox::stateChanged, this, [this, advancedOptionsWidget] (int state) {
        writeParameterToSettings(m_parameters.showAdvancedOptions, state == Qt::Checked, m_settings);
        advancedOptionsWidget->setVisible(state == Qt::Checked);
    });
    connect(compactRadioButton, &QRadioButton::toggled, this, [this, indentedRadioButton] (bool switched) {
        writeParameterToSettings(m_parameters.compactOutput, switched, m_settings);
        indentedRadioButton->setChecked(false);
    });
    connect(indentedRadioButton, &QRadioButton::toggled, this, [this, compactRadioButton] (bool switched) {
        writeParameterToSettings(m_parameters.compactOutput, !switched, m_settings);
        compactRadioButton->setChecked(false);
    });
    connect(keepTimestampsCheckBox, &QCheckBox::stateChanged, this, [this] (int state) {
        writeParameterToSettings(m_parameters.keepTimestamps, state == Qt::Checked, m_settings);
    });

    setFileFormat("json");
}
