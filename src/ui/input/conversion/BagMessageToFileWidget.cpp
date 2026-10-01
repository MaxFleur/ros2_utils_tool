#include "BagMessageToFileWidget.hpp"

#include "UtilsROS.hpp"
#include "UtilsUI.hpp"

#include <QComboBox>
#include <QFormLayout>
#include <QRadioButton>

BagMessageToFileWidget::BagMessageToFileWidget(Parameters::BagMessageToFileParameters& parameters, QWidget *parent) :
    TopicComboBoxWidget(parameters, "Bag Message To File", ":/icons/tools/bag_message_to_file", "Bag File:", "File(s) Location:",
                        "bag_message_to_file", OUTPUT_TYPE::OUTPUT_MESSAGE_TO_FILE, parent),
    m_parameters(parameters), m_settings(parameters, "bag_message_to_file")
{
    m_sourceLineEdit->setToolTip("The source bag file directory.");
    m_targetLineEdit->setToolTip("The target yaml or json file directory.");

    m_basicOptionsFormLayout->insertRow(1, "Topic Name:", m_topicNameComboBox);

    auto* const formatComboBox = new QComboBox;
    formatComboBox->addItem("yaml", 0);
    formatComboBox->addItem("json", 1);
    formatComboBox->setToolTip("The format of the written message files.");
    formatComboBox->setCurrentText(m_parameters.isYaml ? "yaml" : "json");

    m_basicOptionsFormLayout->addRow("Format:", formatComboBox);

    auto* const singleFileRadioButton = new QRadioButton("Single File");
    singleFileRadioButton->setToolTip("Export all topics into a single file.");
    singleFileRadioButton->setChecked(m_parameters.writeSingleOutputFile);

    auto* const multipleFilesRadioButton = new QRadioButton("One File per Topic");
    multipleFilesRadioButton->setToolTip("Export each topic into a separate file.");
    multipleFilesRadioButton->setChecked(!m_parameters.writeSingleOutputFile);

    auto* const optionsLayout = new QFormLayout;
    optionsLayout->addRow("File Structure:", singleFileRadioButton);
    optionsLayout->addRow("", multipleFilesRadioButton);

    m_controlsLayout->addSpacing(10);
    m_controlsLayout->addLayout(optionsLayout);
    m_controlsLayout->addStretch();

    // Generally, enable ok only if we have a source and target directory
    enableOkButton(!m_parameters.sourceDirectory.isEmpty() && !m_parameters.targetDirectory.isEmpty());

    connect(singleFileRadioButton, &QRadioButton::toggled, this, [this, multipleFilesRadioButton] (bool switched) {
        writeParameterToSettings(m_parameters.writeSingleOutputFile, switched, m_settings);
        multipleFilesRadioButton->setChecked(false);
    });
    connect(multipleFilesRadioButton, &QRadioButton::toggled, this, [this, singleFileRadioButton] (bool switched) {
        writeParameterToSettings(m_parameters.writeSingleOutputFile, !switched, m_settings);
        singleFileRadioButton->setChecked(false);
    });
    connect(formatComboBox, &QComboBox::currentTextChanged, this, [this] (const QString& text) {
        writeParameterToSettings(m_parameters.isYaml, text == "yaml", m_settings);
    });
}
