#include "BagToImagesWidget.hpp"

#include "UtilsUI.hpp"

#include <QCheckBox>
#include <QComboBox>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QSlider>

BagToImagesWidget::BagToImagesWidget(Parameters::BagToImagesParameters& parameters, QWidget *parent) :
    TopicComboBoxWidget(parameters, "Bag to Images", ":/icons/bag_to_images", "Bag File:", "Images Location:", "bag_to_images", OUTPUT_IMAGES, parent),
    m_parameters(parameters), m_settings(parameters, "bag_to_images")
{
    m_sourceLineEdit->setToolTip("The source bag file directory.");
    m_topicNameComboBox->setToolTip("The image messages topic.\nIf the bag contains multiple video topics, you can choose one of them.");
    m_targetLineEdit->setToolTip("The target image files directory.");

    auto* const formatComboBox = new QComboBox;
    formatComboBox->addItem("jpg", 0);
    formatComboBox->addItem("png", 1);
    formatComboBox->addItem("bmp", 2);
    formatComboBox->setToolTip("The format of the written images.");
    formatComboBox->setCurrentText(m_parameters.format);

    m_basicOptionsFormLayout->insertRow(1, "Topic Name:", m_topicNameComboBox);
    m_basicOptionsFormLayout->addRow("Format:", formatComboBox);

    auto* const advancedOptionsCheckBox = new QCheckBox;
    advancedOptionsCheckBox->setChecked(m_parameters.showAdvancedOptions);
    advancedOptionsCheckBox->setText("Show Advanced Options");

    auto* const switchRedBlueCheckBox = Utils::UI::createCheckBox("Switch the video's red and blue values.", m_parameters.exchangeRedBlueValues);
    auto* const useBWCheckBox = Utils::UI::createCheckBox("If the images should be colorless or not.", m_parameters.useBWImages);

    m_advancedOptionsFormLayout = new QFormLayout;
    m_advancedOptionsFormLayout->addRow("Switch Red and Blue Values:", switchRedBlueCheckBox);
    m_advancedOptionsFormLayout->addRow("Colorless Images:", useBWCheckBox);

    auto* const advancedOptionsWidget = new QWidget;
    advancedOptionsWidget->setLayout(m_advancedOptionsFormLayout);
    advancedOptionsWidget->setVisible(m_parameters.showAdvancedOptions);

    m_controlsLayout->addWidget(advancedOptionsCheckBox);
    m_controlsLayout->addSpacing(10);
    m_controlsLayout->addWidget(advancedOptionsWidget);
    m_controlsLayout->addStretch();

    adjustWidgetsToChangedFormat(m_parameters.format);

    // Generally, only enable this if the source bag, topic name and target dir line edit contain text
    enableOkButton(!m_parameters.sourceDirectory.isEmpty() &&
                   !m_parameters.topicName.isEmpty() && !m_parameters.targetDirectory.isEmpty());

    connect(formatComboBox, &QComboBox::currentTextChanged, this, &BagToImagesWidget::adjustWidgetsToChangedFormat);
    connect(advancedOptionsCheckBox, &QCheckBox::stateChanged, this, [this, advancedOptionsWidget] (int state) {
        writeParameterToSettings(m_parameters.showAdvancedOptions, state == Qt::Checked, m_settings);
        advancedOptionsWidget->setVisible(state == Qt::Checked);
    });
    connect(switchRedBlueCheckBox, &QCheckBox::stateChanged, this, [this] (int state) {
        writeParameterToSettings(m_parameters.exchangeRedBlueValues, state == Qt::Checked, m_settings);
    });
    connect(useBWCheckBox, &QCheckBox::stateChanged, this, [this] (int state) {
        writeParameterToSettings(m_parameters.useBWImages, state == Qt::Checked, m_settings);
    });
}


// Some parameters also change if the images format is changed, so we need to update the UI accordingly
void
BagToImagesWidget::adjustWidgetsToChangedFormat(const QString& text)
{
    writeParameterToSettings(m_parameters.format, text, m_settings);

    if (m_optimizeOrBilevelCheckBox && m_qualitySlider) {
        m_advancedOptionsFormLayout->removeRow(m_optimizeOrBilevelCheckBox);
        m_advancedOptionsFormLayout->removeRow(m_qualitySlider);
    }
    if (text == "bmp") {
        return;
    }

    m_qualitySlider = new QSlider(Qt::Horizontal);
    m_qualitySlider->setRange(0, 9);
    m_qualitySlider->setTickInterval(1);
    m_qualitySlider->setValue(m_parameters.quality);
    m_qualitySlider->setTickPosition(QSlider::TicksBelow);
    m_qualitySlider->setToolTip(text == "jpg" ? "Image quality. A higher quality will increase the image size."
                                              : "Higher compression will result in smaller size, but increase writing time.");

    m_optimizeOrBilevelCheckBox = new QCheckBox;
    auto& memberVal = text == "jpg" ? m_parameters.jpgOptimize : m_parameters.pngBilevel;
    m_optimizeOrBilevelCheckBox->setChecked(memberVal);
    m_optimizeOrBilevelCheckBox->setToolTip(text == "jpg" ? "Optimize the stored file size." : "Save as an image containing only black and white pixels.");

    m_advancedOptionsFormLayout->insertRow(0, text == "jpg" ? "Quality:" : "Level of Compression:", m_qualitySlider);
    m_advancedOptionsFormLayout->insertRow(1, text == "jpg" ? "Optimize Size" : "Binary Image", m_optimizeOrBilevelCheckBox);

    connect(m_qualitySlider, &QSlider::valueChanged, this, [this] (int value) {
        writeParameterToSettings(m_parameters.quality, value, m_settings);
    });
    connect(m_optimizeOrBilevelCheckBox, &QCheckBox::stateChanged, this, [this] (int state) {
        m_parameters.format == "jpg" ?
        writeParameterToSettings(m_parameters.jpgOptimize, state == Qt::Checked, m_settings) :
        writeParameterToSettings(m_parameters.pngBilevel, state == Qt::Checked, m_settings);
    });
}
