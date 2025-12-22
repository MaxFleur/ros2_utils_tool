#include "TF2ToFileWidget.hpp"

#include "UtilsGeneral.hpp"
#include "UtilsUI.hpp"

#include <QCheckBox>
#include <QComboBox>
#include <QFormLayout>
#include <QLabel>
#include <QRadioButton>

TF2ToFileWidget::TF2ToFileWidget(Parameters::TF2ToFileParameters& parameters, QWidget *parent) :
    TopicComboBoxWidget(parameters, "TF2 to File", ":/icons/tf2_to_file", "Bag File:", "File Location:", "tf2_to_file", OUTPUT_TF_TO_FILE, parent),
    m_parameters(parameters), m_settings(parameters, "tf2_to_file")
{
    m_sourceLineEdit->setToolTip("The source bag file directory.");
    m_topicNameComboBox->setToolTip("The transformation messages topic.\nIf the bag contains multiple transformation topics, you can choose one of them.");
    m_targetLineEdit->setToolTip("The target json file directory.");

    m_basicOptionsFormLayout->insertRow(1, "Topic Name:", m_topicNameComboBox);

    auto* const advancedOptionsCheckBox = new QCheckBox;
    advancedOptionsCheckBox->setChecked(m_parameters.showAdvancedOptions);
    advancedOptionsCheckBox->setText("Show Advanced Options");

    auto* const keepTimestampsCheckBox = Utils::UI::createCheckBox("Keep the message's timestamp into the file.", m_parameters.keepTimestamps);

    m_formatComboBox = new QComboBox;
    m_formatComboBox->addItem("json", 0);
    m_formatComboBox->addItem("yaml", 1);

    const auto isFormatJson = Utils::General::getFileExtension(m_parameters.targetDirectory) == "json";
    m_formatComboBox->setCurrentText(isFormatJson ? "json" : "yaml");

    m_advancedOptionsFormLayout = new QFormLayout;
    m_advancedOptionsFormLayout->addRow("Save Message Timestamps:", keepTimestampsCheckBox);
    m_advancedOptionsFormLayout->addRow("File Format:", m_formatComboBox);
    m_advancedOptionsFormLayout->addRow("", new QLabel(""));

    auto* const advancedOptionsWidget = new QWidget;
    advancedOptionsWidget->setLayout(m_advancedOptionsFormLayout);
    advancedOptionsWidget->setVisible(m_parameters.showAdvancedOptions);

    m_controlsLayout->addWidget(advancedOptionsCheckBox);
    m_controlsLayout->addSpacing(10);
    m_controlsLayout->addWidget(advancedOptionsWidget);
    m_controlsLayout->addStretch();

    // Generally, enable ok only if we have a source and target directory and a topic name
    enableOkButton(!m_parameters.sourceDirectory.isEmpty() &&
                   !m_topicNameComboBox->currentText().isEmpty() && !m_parameters.targetDirectory.isEmpty());

    formatComboBoxTextChanged(isFormatJson);

    connect(advancedOptionsCheckBox, &QCheckBox::stateChanged, this, [this, advancedOptionsWidget] (int state) {
        writeParameterToSettings(m_parameters.showAdvancedOptions, state == Qt::Checked, m_settings);
        advancedOptionsWidget->setVisible(state == Qt::Checked);
    });
    connect(keepTimestampsCheckBox, &QCheckBox::stateChanged, this, [this] (int state) {
        writeParameterToSettings(m_parameters.keepTimestamps, state == Qt::Checked, m_settings);
    });
    connect(m_formatComboBox, &QComboBox::currentTextChanged, this, [this] (const QString& text) {
        formatComboBoxTextChanged(text == "json");
    });

    setFileFormat(isFormatJson ? "json" : "yaml");
}


void
TF2ToFileWidget::formatComboBoxTextChanged(bool switched)
{
    if (switched) {
        m_compactRadioButton = new QRadioButton("Compact");
        m_compactRadioButton->setToolTip("Use a compact json format to keep the file as short as possible.");
        m_compactRadioButton->setChecked(m_parameters.compactOutput);

        m_indentedRadioButton = new QRadioButton("Indented");
        m_indentedRadioButton->setToolTip("Indent the json file for better readability.");
        m_indentedRadioButton->setChecked(!m_parameters.compactOutput);

        m_advancedOptionsFormLayout->addRow("Json Formatting:", m_compactRadioButton);
        m_advancedOptionsFormLayout->addRow("", m_indentedRadioButton);

        connect(m_compactRadioButton, &QRadioButton::toggled, this, [this] (bool switched) {
            writeParameterToSettings(m_parameters.compactOutput, switched, m_settings);
        });
        connect(m_indentedRadioButton, &QRadioButton::toggled, this, [this] (bool switched) {
            writeParameterToSettings(m_parameters.compactOutput, !switched, m_settings);
        });
    } else if (m_compactRadioButton && m_indentedRadioButton) {
        m_advancedOptionsFormLayout->removeRow(m_compactRadioButton);
        m_advancedOptionsFormLayout->removeRow(m_indentedRadioButton);
    }

    if (m_targetLineEdit->text().isEmpty()) {
        return;
    }

    // If the combo box item changes, apply a different appendix to the text in the target line edit
    const auto& formatText = m_formatComboBox->currentText();
    const auto& newLineEditText = Utils::UI::replaceTextAppendix(m_targetLineEdit->text(), formatText);
    m_targetLineEdit->setText(newLineEditText);

    writeParameterToSettings(m_parameters.targetDirectory, newLineEditText, m_settings);
    setFileFormat(formatText);
}
