#include "BagToVideoWidget.hpp"

#include "UtilsUI.hpp"

#include <QCheckBox>
#include <QComboBox>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QSpinBox>

BagToVideoWidget::BagToVideoWidget(Parameters::BagToVideoParameters& parameters, QWidget *parent) :
    TopicComboBoxWidget(parameters, "Bag to Video", ":/icons/bag_to_video", "Bag File:", "Video Location:", "bag_to_video", OUTPUT_VIDEO, parent),
    m_parameters(parameters), m_settings(parameters, "bag_to_video")
{
    m_sourceLineEdit->setToolTip("The source bag file directory.");
    m_topicNameComboBox->setToolTip("The image messages topic.\nIf the bag contains multiple video topics, you can choose one of them.");
    m_targetLineEdit->setToolTip("The target video file directory.");

    m_formatComboBox = new QComboBox;
    m_formatComboBox->addItem("mp4", 0);
    m_formatComboBox->addItem("mkv", 1);
    m_formatComboBox->addItem("avi", 2);
    m_formatComboBox->setToolTip("The video format file.");
    m_formatComboBox->setCurrentText(m_parameters.format);

    m_basicOptionsFormLayout->insertRow(1, "Topic Name:", m_topicNameComboBox);
    m_basicOptionsFormLayout->addRow("Format:", m_formatComboBox);

    auto* const advancedOptionsCheckBox = new QCheckBox;
    advancedOptionsCheckBox->setChecked(m_parameters.showAdvancedOptions);
    advancedOptionsCheckBox->setText("Show Advanced Options");

    auto* const fpsSpinBox = new QSpinBox;
    fpsSpinBox->setRange(10, 60);
    fpsSpinBox->setValue(m_parameters.fps);
    fpsSpinBox->setToolTip("FPS of the encoded video.");

    auto* const switchRedBlueCheckBox = Utils::UI::createCheckBox("Switch the video's red and blue values.", m_parameters.exchangeRedBlueValues);
    auto* const useBWImagesCheckBox = Utils::UI::createCheckBox("Write a colorless video.", m_parameters.useBWImages);

    m_advancedOptionsFormLayout = new QFormLayout;
    m_advancedOptionsFormLayout->addRow("FPS:", fpsSpinBox);
    m_advancedOptionsFormLayout->addRow("Switch Red and Blue Values:", switchRedBlueCheckBox);
    m_advancedOptionsFormLayout->addRow("Use Colorless Images:", useBWImagesCheckBox);

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

    // Call once to potentially enable lossless images checkbox
    formatComboBoxTextChanged(m_formatComboBox->currentText());

    connect(m_formatComboBox, &QComboBox::currentTextChanged, this, &BagToVideoWidget::formatComboBoxTextChanged);
    connect(advancedOptionsCheckBox, &QCheckBox::stateChanged, this, [this, advancedOptionsWidget] (int state) {
        writeParameterToSettings(m_parameters.showAdvancedOptions, state == Qt::Checked, m_settings);
        advancedOptionsWidget->setVisible(state == Qt::Checked);
    });
    connect(fpsSpinBox, QOverload<int>::of(&QSpinBox::valueChanged), this, [this] (int value) {
        writeParameterToSettings(m_parameters.fps, value, m_settings);
    });
    connect(switchRedBlueCheckBox, &QCheckBox::stateChanged, this, [this] (int state) {
        writeParameterToSettings(m_parameters.exchangeRedBlueValues, state == Qt::Checked, m_settings);
    });
    connect(useBWImagesCheckBox, &QCheckBox::stateChanged, this, [this] (int state) {
        writeParameterToSettings(m_parameters.useBWImages, state == Qt::Checked, m_settings);
    });

    setFileFormat(m_formatComboBox->currentText());
}


// Need to adjust some UI elements if the format changes, because options differ between formats
void
BagToVideoWidget::formatComboBoxTextChanged(const QString& text)
{
    writeParameterToSettings(m_parameters.format, text, m_settings);

    if (m_useLosslessCheckBox) {
        m_advancedOptionsFormLayout->removeRow(m_useLosslessCheckBox);
    }
    if (text == "mkv" || text == "avi") {
        m_useLosslessCheckBox = new QCheckBox;
        m_useLosslessCheckBox->setToolTip("If the video images should be lossless. Improves video quality, but increases file size.");
        m_useLosslessCheckBox->setCheckState(m_parameters.lossless ? Qt::Checked : Qt::Unchecked);

        m_advancedOptionsFormLayout->addRow("Lossless Video:", m_useLosslessCheckBox);

        connect(m_useLosslessCheckBox, &QCheckBox::stateChanged, this, [this] (int state) {
            m_parameters.lossless = state == Qt::Checked;
            m_settings.write();
        });
    }

    if (m_targetLineEdit->text().isEmpty()) {
        return;
    }

    // If the combo box item changes, apply a different appendix to the text in the video line edit
    const auto& newLineEditText = Utils::UI::replaceTextAppendix(m_targetLineEdit->text(), text);
    m_targetLineEdit->setText(newLineEditText);

    writeParameterToSettings(m_parameters.targetDirectory, newLineEditText, m_settings);
    setFileFormat(text);
}
