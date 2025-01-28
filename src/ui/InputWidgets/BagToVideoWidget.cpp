#include "BagToVideoWidget.hpp"

#include "UtilsROS.hpp"

#include <QCheckBox>
#include <QComboBox>
#include <QDialogButtonBox>
#include <QEvent>
#include <QFileDialog>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QShortcut>
#include <QSpinBox>
#include <QToolButton>
#include <QVBoxLayout>

#include <filesystem>

BagToVideoWidget::BagToVideoWidget(Utils::UI::VideoInputParameters& parameters, QWidget *parent) :
    BasicInputWidget("Encode Video from ROSBag", ":/icons/bag_to_video", parent),
    m_parameters(parameters), m_settings(parameters, "bag_to_video")
{
    m_sourceLineEdit->setText(parameters.sourceDirectory);
    m_sourceLineEdit->setToolTip("The directory of the ROSBag source file.");

    m_topicNameComboBox = new QComboBox;
    m_topicNameComboBox->setMinimumWidth(200);
    m_topicNameComboBox->setToolTip("The ROSBag topic of the video file.\n"
                                    "If the Bag contains multiple video topics, you can choose one of them.");
    if (!m_parameters.sourceDirectory.isEmpty()) {
        Utils::UI::fillComboBoxWithTopics(m_topicNameComboBox, m_parameters.sourceDirectory);

        if (!m_parameters.topicName.isEmpty()) {
            m_topicNameComboBox->setCurrentText(m_parameters.topicName);
        }
    }

    m_videoNameLineEdit = new QLineEdit(m_parameters.targetDirectory);
    m_videoNameLineEdit->setToolTip("The directory where the video file should be stored.");

    auto* const videoLocationButton = new QToolButton;
    auto* const searchVideoPathLayout = Utils::UI::createLineEditButtonLayout(m_videoNameLineEdit, videoLocationButton);

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
    auto* const switchRedBlueCheckBox = Utils::UI::createCheckBox("Switch the video's red and blue values.", m_parameters.switchRedBlueValues);
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

    auto* const okShortCut = new QShortcut(QKeySequence(Qt::Key_Return), this);
    enableOkButton(!m_parameters.sourceDirectory.isEmpty() &&
                   !m_parameters.topicName.isEmpty() && !m_parameters.targetDirectory.isEmpty());

    // Call once to potentially enable lossless images checkbox
    formatComboBoxTextChanged(m_formatComboBox->currentText());

    connect(m_findSourceButton, &QPushButton::clicked, this, &BagToVideoWidget::searchButtonPressed);
    connect(videoLocationButton, &QPushButton::clicked, this, &BagToVideoWidget::videoLocationButtonPressed);
    connect(m_topicNameComboBox, &QComboBox::currentTextChanged, this, [this] (const QString& text) {
        writeSettingsParameter(m_parameters.topicName, text, m_settings);
    });
    connect(m_formatComboBox, &QComboBox::currentTextChanged, this, &BagToVideoWidget::formatComboBoxTextChanged);
    connect(advancedOptionsCheckBox, &QCheckBox::stateChanged, this, [this, advancedOptionsWidget] (int state) {
        m_parameters.showAdvancedOptions = state == Qt::Checked;
        advancedOptionsWidget->setVisible(state == Qt::Checked);
    });
    connect(fpsSpinBox, QOverload<int>::of(&QSpinBox::valueChanged), this, [this] (int value) {
        writeSettingsParameter(m_parameters.fps, value, m_settings);
    });
    connect(useHardwareAccCheckBox, &QCheckBox::stateChanged, this, [this] (int state) {
        writeSettingsParameter(m_parameters.useHardwareAcceleration, state == Qt::Checked, m_settings);
    });
    connect(switchRedBlueCheckBox, &QCheckBox::stateChanged, this, [this] (int state) {
        writeSettingsParameter(m_parameters.switchRedBlueValues, state == Qt::Checked, m_settings);
    });
    connect(useBWImagesCheckBox, &QCheckBox::stateChanged, this, [this] (int state) {
        writeSettingsParameter(m_parameters.useBWImages, state == Qt::Checked, m_settings);
    });
    connect(m_dialogButtonBox, &QDialogButtonBox::accepted, this, &BagToVideoWidget::okButtonPressed);
    connect(okShortCut, &QShortcut::activated, this, &BagToVideoWidget::okButtonPressed);
}


void
BagToVideoWidget::searchButtonPressed()
{
    const auto bagDirectory = QFileDialog::getExistingDirectory(this, "Open ROSBag", "",
                                                                QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
    if (bagDirectory.isEmpty()) {
        return;
    }

    if (const auto containsVideoTopics = Utils::UI::fillComboBoxWithTopics(m_topicNameComboBox, bagDirectory); !containsVideoTopics) {
        Utils::UI::createCriticalMessageBox("Topic not found!", "The bag file does not contain any image/video topics!");
        return;
    }

    m_sourceLineEdit->setText(bagDirectory);
    writeSettingsParameter(m_parameters.sourceDirectory, bagDirectory, m_settings);

    QDir bagDirectoryDir(bagDirectory);
    bagDirectoryDir.cdUp();
    if (const auto autoVideoDirectory = bagDirectoryDir.path() + "/bag_video." + m_formatComboBox->currentText();
        !std::filesystem::exists(autoVideoDirectory.toStdString())) {
        m_videoNameLineEdit->setText(autoVideoDirectory);
        writeSettingsParameter(m_parameters.targetDirectory, autoVideoDirectory, m_settings);
    }

    // Only enable if both line edits contain text
    enableOkButton(!m_parameters.sourceDirectory.isEmpty() &&
                   !m_parameters.topicName.isEmpty() && !m_parameters.targetDirectory.isEmpty());
}


void
BagToVideoWidget::videoLocationButtonPressed()
{
    const auto fileName = QFileDialog::getSaveFileName(this, "Save Video", "",
                                                       m_formatComboBox->currentText() + " files (*." + m_formatComboBox->currentText() + ")");
    if (fileName.isEmpty()) {
        return;
    }

    // Only enable if both line edits contain text
    m_fileDialogOpened = true;
    writeSettingsParameter(m_parameters.targetDirectory, fileName, m_settings);
    m_videoNameLineEdit->setText(fileName);
    enableOkButton(!m_parameters.sourceDirectory.isEmpty() &&
                   !m_parameters.topicName.isEmpty() && !m_parameters.targetDirectory.isEmpty());
}


void
BagToVideoWidget::formatComboBoxTextChanged(const QString& text)
{
    writeSettingsParameter(m_parameters.format, text, m_settings);

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

    // If the combo box item changes, apply a different appendix to the text in the video line edit
    if (m_videoNameLineEdit->text().isEmpty()) {
        return;
    }

    auto newLineEditText = m_videoNameLineEdit->text();
    newLineEditText.truncate(newLineEditText.lastIndexOf(QChar('.')));
    newLineEditText += "." + text;
    m_videoNameLineEdit->setText(newLineEditText);

    writeSettingsParameter(m_parameters.targetDirectory, newLineEditText, m_settings);
}


void
BagToVideoWidget::okButtonPressed()
{
    if (!m_okButton->isEnabled()) {
        return;
    }

    // Only ask if exists and the file dialog has not been called
    if (std::filesystem::exists(m_videoNameLineEdit->text().toStdString()) && !m_fileDialogOpened) {
        auto *const msgBox = new QMessageBox(QMessageBox::Warning, "Video already exists!",
                                             "A video already exists under the specified directory! Are you sure you want to continue? This will overwrite the existing file.",
                                             QMessageBox::Yes | QMessageBox::No);
        if (const auto ret = msgBox->exec(); ret == QMessageBox::No) {
            return;
        }
    }

    emit okPressed();
}
