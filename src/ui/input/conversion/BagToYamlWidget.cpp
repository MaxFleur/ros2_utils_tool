#include "BagToYamlWidget.hpp"

#include "UtilsROS.hpp"
#include "UtilsUI.hpp"

#include <QFormLayout>
#include <QRadioButton>

BagToYamlWidget::BagToYamlWidget(Parameters::BagToYamlParameters& parameters, QWidget *parent) :
    TopicComboBoxWidget(parameters, "Bag To File", ":/icons/tools/bag_to_yaml", "Bag File:", "File(s) Location:", "bag_to_yaml", OUTPUT_TYPE::OUTPUT_YAML, parent),
    m_parameters(parameters), m_settings(parameters, "bag_to_yaml")
{
    m_sourceLineEdit->setToolTip("The source bag file directory.");
    m_targetLineEdit->setToolTip("The target YAML file directory.");

    m_basicOptionsFormLayout->insertRow(1, "Topic Name:", m_topicNameComboBox);

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

    setFileFormat("yaml");
}
