#include "VideoToBagWidget.hpp"

#include "UtilsROS.hpp"
#include "UtilsUI.hpp"

#include <QCheckBox>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QRadioButton>
#include <QSpinBox>

#include <filesystem>

VideoToBagWidget::VideoToBagWidget(Parameters::VideoToBagParameters& parameters,
                                   bool usePredefinedTopicName, bool warnROS2NameConvention, QWidget *parent) :
    TopicComboBoxWidget(parameters, "Video to Bag", ":/icons/tools/video_to_bag", "Video Dir:", "Bag Location:", "vid_to_bag", OUTPUT_TYPE::OUTPUT_BAG, parent),
    m_parameters(parameters), m_settings(parameters, "vid_to_bag"),
    m_warnROS2NameConvention(warnROS2NameConvention)
{
    m_sourceLineEdit->setToolTip("The source video file directory.");
    m_targetLineEdit->setToolTip("The target bag file directory.");

    auto* const topicNameLineEdit = new QLineEdit(m_parameters.topicName);
    topicNameLineEdit->setToolTip("The video's topic name inside the source bag.");
    // Use a predefined name if set in the settings
    if (usePredefinedTopicName && m_parameters.topicName.isEmpty()) {
        topicNameLineEdit->setText("/topic_video");
        writeParameterToSettings(m_parameters.topicName, topicNameLineEdit->text(), m_settings);
    }

    m_basicOptionsFormLayout->insertRow(1, "Topic Name:", topicNameLineEdit);

    auto* const switchRedBlueCheckBox = Utils::UI::createCheckBox("Switch the video's red and blue values.", m_parameters.exchangeRedBlueValues);
    auto* const useCustomFPSCheckBox = Utils::UI::createCheckBox("Use custom fps for the bag file. If this is unchecked, "
                                                                 "the video's fps will be used.", m_parameters.useCustomFPS);
    auto* const useCompressionCheckBox = Utils::UI::createCheckBox("Use compressed image messages. This decreases the bag's size, "
                                                                   "but writing takes longer.", m_parameters.useCompression);

    m_advancedOptionsFormLayout = new QFormLayout;
    m_advancedOptionsFormLayout->addRow("Switch Red and Blue Values:", switchRedBlueCheckBox);
    m_advancedOptionsFormLayout->addRow("Use Custom FPS:", useCustomFPSCheckBox);
    m_advancedOptionsFormLayout->addRow("Use Compression:", useCompressionCheckBox);

    createAdvancedOptionsWidget(m_advancedOptionsFormLayout, m_parameters.showAdvancedOptions,
                                [this] (bool shown) {
        writeParameterToSettings(m_parameters.showAdvancedOptions, shown, m_settings);
    });

    enableOkButton(!m_parameters.sourceDirectory.isEmpty() && !m_parameters.targetDirectory.isEmpty() && !m_parameters.topicName.isEmpty());

    connect(topicNameLineEdit, &QLineEdit::textChanged, this, [this, topicNameLineEdit] {
        writeParameterToSettings(m_parameters.topicName, topicNameLineEdit->text(), m_settings);
        enableOkButton(!m_parameters.sourceDirectory.isEmpty() && !m_parameters.targetDirectory.isEmpty() && !m_parameters.topicName.isEmpty());
    });
    connect(switchRedBlueCheckBox, &QCheckBox::stateChanged, this, [this] (int state) {
        writeParameterToSettings(m_parameters.exchangeRedBlueValues, state == Qt::Checked, m_settings);
    });
    connect(useCustomFPSCheckBox, &QCheckBox::stateChanged, this, &VideoToBagWidget::useCustomFPSCheckBoxPressed);
    connect(useCompressionCheckBox, &QCheckBox::stateChanged, this, &VideoToBagWidget::useCompressionCheckBoxPressed);

    useCustomFPSCheckBoxPressed(m_parameters.useCustomFPS);
    useCompressionCheckBoxPressed(m_parameters.useCompression);
}


void
VideoToBagWidget::findSourceButtonPressed()
{
    enableOkButton(false);

    const auto videoDir = QFileDialog::getOpenFileName(this, "Open Video");
    if (videoDir.isEmpty()) {
        return;
    }

    QFileInfo fileInfo(videoDir);
    if (fileInfo.suffix().toLower() != "mp4" && fileInfo.suffix().toLower() != "mkv" && fileInfo.suffix().toLower() != "avi") {
        auto *const msgBox = new QMessageBox(QMessageBox::Critical, "Wrong format!", "The video must be in mp4 or mkv format!", QMessageBox::Ok);
        msgBox->setAttribute(Qt::WA_DeleteOnClose);
        msgBox->exec();

        return;
    }

    writeParameterToSettings(m_parameters.sourceDirectory, videoDir, m_settings);
    m_sourceLineEdit->setText(videoDir);
    // Create bag file name automatically so the user has to put in less values
    QDir videoDirectoryDir(videoDir);
    videoDirectoryDir.cdUp();
    if (const auto autoBagDirectory = videoDirectoryDir.path() + "/video_bag"; !std::filesystem::exists(autoBagDirectory.toStdString())) {
        m_targetLineEdit->setText(autoBagDirectory);

        writeParameterToSettings(m_parameters.targetDirectory, autoBagDirectory, m_settings);
        setLowDiskSpaceWidgetVisibility(m_targetLineEdit->text());
    }

    enableOkButton(!m_parameters.sourceDirectory.isEmpty() && !m_parameters.targetDirectory.isEmpty() && !m_parameters.topicName.isEmpty());
}


// We only need to create this if we want to use a custom fps value
void
VideoToBagWidget::useCustomFPSCheckBoxPressed(int state)
{
    // Partially checked value can still count for this case
    writeParameterToSettings(m_parameters.useCustomFPS, state != Qt::Unchecked, m_settings);

    if (state != Qt::Unchecked) {
        m_fpsSpinBox = new QSpinBox;
        m_fpsSpinBox->setRange(10, 60);
        m_fpsSpinBox->setValue(m_parameters.fps);
        m_fpsSpinBox->setToolTip("FPS of the video stored in the bag.");

        m_advancedOptionsFormLayout->insertRow(ADVANCED_OPTIONS_LAYOUT_ROW_THIRD, "", m_fpsSpinBox);

        connect(m_fpsSpinBox, QOverload<int>::of(&QSpinBox::valueChanged), this, [this] (int value) {
            writeParameterToSettings(m_parameters.fps, value, m_settings);
        });
    } else if (m_fpsSpinBox) {
        m_advancedOptionsFormLayout->removeRow(m_fpsSpinBox);
    }
}


// Same here
void
VideoToBagWidget::useCompressionCheckBoxPressed(int state)
{
    // Partially checked value can still count for this case
    writeParameterToSettings(m_parameters.useCompression, state != Qt::Unchecked, m_settings);

    if (state != Qt::Unchecked) {
        m_compressJPEGRadioButton = new QRadioButton("JPEG");
        m_compressPNGRadioButton = new QRadioButton("PNG");

        m_advancedOptionsFormLayout->insertRow(m_parameters.useCustomFPS ? ADVANCED_OPTIONS_LAYOUT_ROW_FIFTH : ADVANCED_OPTIONS_LAYOUT_ROW_FOURTH, "", m_compressJPEGRadioButton);
        m_advancedOptionsFormLayout->insertRow(m_parameters.useCustomFPS ? ADVANCED_OPTIONS_LAYOUT_ROW_SIXTH : ADVANCED_OPTIONS_LAYOUT_ROW_FIFTH, "", m_compressPNGRadioButton);

        m_parameters.isCompressionJPEG ? m_compressJPEGRadioButton->click() : m_compressPNGRadioButton->click();

        connect(m_compressJPEGRadioButton, &QPushButton::clicked, this, [this] {
            writeParameterToSettings(m_parameters.isCompressionJPEG, true, m_settings);
        });
        connect(m_compressPNGRadioButton, &QPushButton::clicked, this, [this] {
            writeParameterToSettings(m_parameters.isCompressionJPEG, false, m_settings);
        });
    } else if (m_compressJPEGRadioButton && m_compressPNGRadioButton) {
        m_advancedOptionsFormLayout->removeRow(m_compressJPEGRadioButton);
        m_advancedOptionsFormLayout->removeRow(m_compressPNGRadioButton);
    }
}


bool
VideoToBagWidget::areInputsValid() const
{
    if (const auto sufficientSpace = showLowDiskSpaceMessageBox(); !sufficientSpace) {
        return false;
    }
    if (!confirmTopicName(m_parameters.topicName, m_warnROS2NameConvention)) {
        return false;
    }

    return Utils::UI::continueForExistingTarget(m_parameters.targetDirectory, "Bag file", "bag file");
}
