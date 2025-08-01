#include "PublishWidget.hpp"

#include "UtilsROS.hpp"
#include "UtilsUI.hpp"

#include <QCheckBox>
#include <QDialogButtonBox>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QShortcut>
#include <QSpinBox>
#include <QVBoxLayout>

#include <filesystem>

PublishWidget::PublishWidget(Parameters::PublishParameters& parameters, bool usePredefinedTopicName,
                             bool warnROS2NameConvention, bool publishVideo, QWidget *parent) :
    BasicInputWidget(publishVideo ? "Publish Video as ROS Topic" : "Publish Images as ROS Topic",
                     publishVideo ? ":/icons/publish_video" : ":/icons/publish_images", parent),
    m_parameters(parameters), m_settings(parameters, publishVideo ? "publish_video" : "publish_images"),
    m_warnROS2NameConvention(warnROS2NameConvention), m_publishVideo(publishVideo)
{
    m_sourceLineEdit->setText(parameters.sourceDirectory);
    m_sourceLineEdit->setToolTip(m_publishVideo ? "The source video file directory." : "The source images file directory.");

    auto* const topicNameLineEdit = new QLineEdit(m_parameters.topicName);
    topicNameLineEdit->setToolTip("Name of the topic to be published.");
    // Use a predefined name if set in the settings
    if (usePredefinedTopicName && m_parameters.topicName.isEmpty()) {
        topicNameLineEdit->setText("/topic_video");
        writeParameterToSettings(m_parameters.topicName, topicNameLineEdit->text(), m_settings);
    }

    auto* const basicOptionsFormLayout = new QFormLayout;
    basicOptionsFormLayout->addRow(m_publishVideo ? "Video File:" : "Images Files:", m_findSourceLayout);
    basicOptionsFormLayout->addRow("Topic Name:", topicNameLineEdit);

    auto* const advancedOptionsCheckBox = new QCheckBox;
    advancedOptionsCheckBox->setChecked(m_parameters.showAdvancedOptions);
    advancedOptionsCheckBox->setText("Show Advanced Options");

    auto* const scaleCheckBox = Utils::UI::createCheckBox("Scale the video to another width and height.", m_parameters.scale);
    auto* const switchRedBlueCheckBox = Utils::UI::createCheckBox("Switch the video's red and blue values.", m_parameters.exchangeRedBlueValues);
    auto* const loopCheckBox = Utils::UI::createCheckBox("Loop the video file if it has been played through.", m_parameters.loop);

    m_advancedOptionsFormLayout = new QFormLayout;
    m_advancedOptionsFormLayout->addRow("Scale Video:", scaleCheckBox);
    m_advancedOptionsFormLayout->addRow("Switch Red and Blue Values:", switchRedBlueCheckBox);
    m_advancedOptionsFormLayout->addRow("Loop Video:", loopCheckBox);

    // Different input widgets are needed depending on if we want to publish a video or image sequences
    if (!m_publishVideo) {
        auto* const fpsSpinBox = new QSpinBox;
        fpsSpinBox->setRange(1, 60);
        fpsSpinBox->setValue(m_parameters.fps);
        fpsSpinBox->setToolTip("Set the fps value used for publishing.");

        m_advancedOptionsFormLayout->addRow("FPS:", fpsSpinBox);
        connect(fpsSpinBox, QOverload<int>::of(&QSpinBox::valueChanged), this, [this] (int value) {
            writeParameterToSettings(m_parameters.fps, value, m_settings);
        });
    }

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
    enableOkButton(!m_parameters.sourceDirectory.isEmpty() && !m_parameters.topicName.isEmpty());

    connect(m_findSourceButton, &QPushButton::clicked, this, &PublishWidget::searchButtonPressed);
    connect(topicNameLineEdit, &QLineEdit::textChanged, this, [this, topicNameLineEdit] {
        writeParameterToSettings(m_parameters.topicName, topicNameLineEdit->text(), m_settings);
        enableOkButton(!m_parameters.sourceDirectory.isEmpty() && !m_parameters.topicName.isEmpty());
    });
    connect(advancedOptionsCheckBox, &QCheckBox::stateChanged, this, [this, advancedOptionsWidget] (int state) {
        writeParameterToSettings(m_parameters.showAdvancedOptions, state == Qt::Checked, m_settings);
        advancedOptionsWidget->setVisible(state == Qt::Checked);
    });
    connect(scaleCheckBox, &QCheckBox::stateChanged, this, &PublishWidget::scaleCheckBoxPressed);
    connect(switchRedBlueCheckBox, &QCheckBox::stateChanged, this, [this] (int state) {
        writeParameterToSettings(m_parameters.exchangeRedBlueValues, state == Qt::Checked, m_settings);
    });
    connect(loopCheckBox, &QCheckBox::stateChanged, this, [this] (int state) {
        writeParameterToSettings(m_parameters.loop, state == Qt::Checked, m_settings);
    });
    connect(m_dialogButtonBox, &QDialogButtonBox::accepted, this, &PublishWidget::okButtonPressed);
    connect(okShortCut, &QShortcut::activated, this, &PublishWidget::okButtonPressed);

    scaleCheckBoxPressed(m_parameters.scale);
}


void
PublishWidget::searchButtonPressed()
{
    const auto dir = m_publishVideo ? QFileDialog::getOpenFileName(this, "Open Video")
                                    : QFileDialog::getExistingDirectory(this, "Open Images", "", QFileDialog::ShowDirsOnly);
    if (dir.isEmpty()) {
        return;
    }

    QFileInfo fileInfo(dir);
    // Check for correct corresponding formats
    // For video: mp4 or mkv
    if (m_publishVideo) {
        if (fileInfo.suffix().toLower() != "mp4" && fileInfo.suffix().toLower() != "mkv") {
            auto *const msgBox = new QMessageBox(QMessageBox::Critical, "Wrong format!", "The video must be in mp4 or mkv format!", QMessageBox::Ok);
            msgBox->exec();
            return;
        }
        // For images: jpg, png or bmp
    } else {
        auto containsImageFiles = false;
        for (auto const& entry : std::filesystem::directory_iterator(dir.toStdString())) {
            if (entry.path().extension() == ".jpg" || entry.path().extension() == ".png" || entry.path().extension() == ".bmp") {
                containsImageFiles = true;
                break;
            }
        }
        if (!containsImageFiles) {
            auto *const msgBox = new QMessageBox(QMessageBox::Critical, "No images!", "The directory does not contain any images!", QMessageBox::Ok);
            msgBox->exec();
            return;
        }
    }

    writeParameterToSettings(m_parameters.sourceDirectory, dir, m_settings);
    m_sourceLineEdit->setText(dir);

    enableOkButton(!m_parameters.sourceDirectory.isEmpty() && !m_parameters.topicName.isEmpty());
}


// We only need to create this if we want to scale the video
void
PublishWidget::scaleCheckBoxPressed(int state)
{
    // Partially checked value can still count for this case
    writeParameterToSettings(m_parameters.scale, state != Qt::Unchecked, m_settings);

    if (state != Qt::Unchecked) {
        m_widthSpinBox = new QSpinBox;
        m_widthSpinBox->setRange(1, 3840);
        m_widthSpinBox->setValue(m_parameters.width);

        m_heightSpinBox = new QSpinBox;
        m_heightSpinBox->setRange(1, 2160);
        m_heightSpinBox->setValue(m_parameters.height);

        m_advancedOptionsFormLayout->insertRow(1, "Width:", m_widthSpinBox);
        m_advancedOptionsFormLayout->insertRow(2, "Height:", m_heightSpinBox);

        connect(m_widthSpinBox, QOverload<int>::of(&QSpinBox::valueChanged), this, [this] (int value) {
            writeParameterToSettings(m_parameters.width, value, m_settings);
        });
        connect(m_heightSpinBox, QOverload<int>::of(&QSpinBox::valueChanged), this, [this] (int value) {
            writeParameterToSettings(m_parameters.height, value, m_settings);
        });
    } else if (m_widthSpinBox && m_heightSpinBox) {
        m_advancedOptionsFormLayout->removeRow(m_widthSpinBox);
        m_advancedOptionsFormLayout->removeRow(m_heightSpinBox);
    }
}


void
PublishWidget::okButtonPressed() const
{
    if (!m_okButton->isEnabled()) {
        return;
    }

    if (m_warnROS2NameConvention && !Utils::ROS::isNameROS2Conform(m_parameters.topicName)) {
        if (const auto returnValue = Utils::UI::continueWithInvalidROS2Names(); !returnValue) {
            return;
        }
    }

    emit okPressed();
}
