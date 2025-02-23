#include "BagToVideoWidget.hpp"

#include <QCheckBox>
#include <QComboBox>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QSpinBox>
#include <QToolButton>
#include <QVBoxLayout>

BagToVideoWidget::BagToVideoWidget(Utils::UI::BagToVideoParameters& parameters, QWidget *parent) :
    AdvancedInputWidget(parameters, "Bag to Video", ":/icons/bag_to_video", "bag_to_video", OUTPUT_VIDEO, parent),
    m_parameters(parameters), m_settings(parameters, "bag_to_video")
{
    m_topicNameComboBox->setToolTip("The image messages topic.\nIf the bag contains multiple video topics, you can choose one of them.");
    m_targetLineEdit->setToolTip("The directory where the video file should be stored.");

    auto* const videoLocationButton = new QToolButton;
    auto* const searchVideoPathLayout = Utils::UI::createLineEditButtonLayout(m_targetLineEdit, videoLocationButton);

    m_formatComboBox = new QComboBox;
    m_formatComboBox->addItem("mp4", 0);
    m_formatComboBox->addItem("mkv", 1);
    m_formatComboBox->setToolTip("The video format file.");
    m_formatComboBox->setCurrentText(m_parameters.format);

    auto* const basicOptionsFormLayout = new QFormLayout;
    basicOptionsFormLayout->addRow("Bag File:", m_findSourceLayout);
    basicOptionsFormLayout->addRow("Topic Name:", m_topicNameComboBox);
    basicOptionsFormLayout->addRow("Video Location:", searchVideoPathLayout);
    basicOptionsFormLayout->addRow("Format:", m_formatComboBox);

    auto* const advancedOptionsCheckBox = new QCheckBox;
    advancedOptionsCheckBox->setChecked(m_parameters.showAdvancedOptions ? Qt::Checked : Qt::Unchecked);
    advancedOptionsCheckBox->setText("Show Advanced Options");

    auto* const fpsSpinBox = new QSpinBox;
    fpsSpinBox->setRange(10, 60);
    fpsSpinBox->setValue(m_parameters.fps);
    fpsSpinBox->setToolTip("FPS of the encoded video.");

    auto* const useHardwareAccCheckBox = Utils::UI::createCheckBox("Enable hardware acceleration for faster video encoding.",
                                                                   m_parameters.useHardwareAcceleration);
    auto* const switchRedBlueCheckBox = Utils::UI::createCheckBox("Switch the video's red and blue values.", m_parameters.exchangeRedBlueValues);
    auto* const useBWImagesCheckBox = Utils::UI::createCheckBox("Write a colorless video.", m_parameters.useBWImages);

    m_advancedOptionsFormLayout = new QFormLayout;
    m_advancedOptionsFormLayout->addRow("FPS:", fpsSpinBox);
    m_advancedOptionsFormLayout->addRow("HW Acceleration:", useHardwareAccCheckBox);
    m_advancedOptionsFormLayout->addRow("Switch Red and Blue Values:", switchRedBlueCheckBox);
    m_advancedOptionsFormLayout->addRow("Use Colorless Images:", useBWImagesCheckBox);

    auto* const advancedOptionsWidget = new QWidget;
    advancedOptionsWidget->setLayout(m_advancedOptionsFormLayout);
    advancedOptionsWidget->setVisible(m_parameters.showAdvancedOptions);

    auto* const controlsLayout = new QVBoxLayout;
    controlsLayout->addStretch();
    controlsLayout->addWidget(m_headerPixmapLabel);
    controlsLayout->addWidget(m_headerLabel);
    controlsLayout->addSpacing(40);
    controlsLayout->addLayout(basicOptionsFormLayout);
    controlsLayout->addSpacing(5);
    controlsLayout->addWidget(advancedOptionsCheckBox);
    controlsLayout->addSpacing(10);
    controlsLayout->addWidget(advancedOptionsWidget);
    controlsLayout->addStretch();

    auto* const controlsSqueezedLayout = new QHBoxLayout;
    controlsSqueezedLayout->addStretch();
    controlsSqueezedLayout->addLayout(controlsLayout);
    controlsSqueezedLayout->addStretch();

    auto* const mainLayout = new QVBoxLayout;
    mainLayout->addLayout(controlsSqueezedLayout);
    mainLayout->addLayout(m_buttonLayout);
    setLayout(mainLayout);

    // Generally, enable ok only if we have a source and target directory and a topic name
    enableOkButton(!m_parameters.sourceDirectory.isEmpty() &&
                   !m_parameters.topicName.isEmpty() && !m_parameters.targetDirectory.isEmpty());

    // Call once to potentially enable lossless images checkbox
    formatComboBoxTextChanged(m_formatComboBox->currentText());

    connect(videoLocationButton, &QPushButton::clicked, this, &BagToVideoWidget::targetLocationButtonPressed);
    connect(m_formatComboBox, &QComboBox::currentTextChanged, this, &BagToVideoWidget::formatComboBoxTextChanged);
    connect(advancedOptionsCheckBox, &QCheckBox::stateChanged, this, [this, advancedOptionsWidget] (int state) {
        m_parameters.showAdvancedOptions = state == Qt::Checked;
        advancedOptionsWidget->setVisible(state == Qt::Checked);
    });
    connect(fpsSpinBox, QOverload<int>::of(&QSpinBox::valueChanged), this, [this] (int value) {
        writeParameterToSettings(m_parameters.fps, value, m_settings);
    });
    connect(useHardwareAccCheckBox, &QCheckBox::stateChanged, this, [this] (int state) {
        writeParameterToSettings(m_parameters.useHardwareAcceleration, state == Qt::Checked, m_settings);
    });
    connect(switchRedBlueCheckBox, &QCheckBox::stateChanged, this, [this] (int state) {
        writeParameterToSettings(m_parameters.exchangeRedBlueValues, state == Qt::Checked, m_settings);
    });
    connect(useBWImagesCheckBox, &QCheckBox::stateChanged, this, [this] (int state) {
        writeParameterToSettings(m_parameters.useBWImages, state == Qt::Checked, m_settings);
    });

    setVideoFormat(m_formatComboBox->currentText());
}


// Need to adjust some UI elements if the format changes, because options differ between formats
void
BagToVideoWidget::formatComboBoxTextChanged(const QString& text)
{
    writeParameterToSettings(m_parameters.format, text, m_settings);

    if (m_useLosslessCheckBox) {
        m_advancedOptionsFormLayout->removeRow(m_useLosslessCheckBox);
    }
    if (text == "mkv") {
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
    auto newLineEditText = m_targetLineEdit->text();
    newLineEditText.truncate(newLineEditText.lastIndexOf(QChar('.')));
    newLineEditText += "." + text;
    m_targetLineEdit->setText(newLineEditText);

    writeParameterToSettings(m_parameters.targetDirectory, newLineEditText, m_settings);
    setVideoFormat(text);
}
