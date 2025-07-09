#include "AdvancedInputWidget.hpp"

#include "LowDiskSpaceWidget.hpp"
#include "UtilsROS.hpp"
#include "UtilsUI.hpp"

#include <QComboBox>
#include <QDialogButtonBox>
#include <QFileDialog>
#include <QFormLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QShortcut>
#include <QToolButton>
#include <QVBoxLayout>

#include <filesystem>

AdvancedInputWidget::AdvancedInputWidget(Parameters::AdvancedParameters& parameters, const QString& headerText,
                                         const QString& iconPath, const QString& sourceFormLayoutName, const QString& targetFormLayoutName,
                                         const QString& settingsIdentifier, int outputFormat, QWidget *parent) :
    BasicInputWidget(headerText, iconPath, parent),
    m_parameters(parameters), m_settings(parameters, settingsIdentifier), m_outputFormat(outputFormat)
{
    m_sourceLineEdit->setText(parameters.sourceDirectory);

    m_topicNameComboBox = new QComboBox;
    m_topicNameComboBox->setMinimumWidth(200);

    if (!m_parameters.sourceDirectory.isEmpty()) {
        Utils::UI::fillComboBoxWithTopics(m_topicNameComboBox, m_parameters.sourceDirectory,
                                          m_outputFormat == OUTPUT_PCDS ? "sensor_msgs/msg/PointCloud2" : "sensor_msgs/msg/Image");

        if (!m_parameters.topicName.isEmpty()) {
            m_topicNameComboBox->setCurrentText(m_parameters.topicName);
        }
    }

    m_targetLineEdit = new QLineEdit(m_parameters.targetDirectory);
    auto* const findTargetButton = new QToolButton;
    auto* const targetLocationLayout = Utils::UI::createLineEditButtonLayout(m_targetLineEdit, findTargetButton);

    m_basicOptionsFormLayout = new QFormLayout;
    m_basicOptionsFormLayout->addRow(sourceFormLayoutName, m_findSourceLayout);
    m_basicOptionsFormLayout->addRow(targetFormLayoutName, targetLocationLayout);

    m_lowDiskSpaceWidget = new LowDiskSpaceWidget;

    m_controlsLayout = new QVBoxLayout;
    m_controlsLayout->addStretch();
    m_controlsLayout->addWidget(m_headerPixmapLabel);
    m_controlsLayout->addWidget(m_headerLabel);
    m_controlsLayout->addSpacing(40);
    m_controlsLayout->addLayout(m_basicOptionsFormLayout);
    m_controlsLayout->addSpacing(5);
    m_controlsLayout->addWidget(m_lowDiskSpaceWidget);
    m_controlsLayout->addSpacing(5);

    auto* const controlsSqueezedLayout = new QHBoxLayout;
    controlsSqueezedLayout->addStretch();
    controlsSqueezedLayout->addLayout(m_controlsLayout);
    controlsSqueezedLayout->addStretch();

    auto* const mainLayout = new QVBoxLayout;
    mainLayout->addLayout(controlsSqueezedLayout);
    mainLayout->addLayout(m_buttonLayout);
    setLayout(mainLayout);

    auto* const okShortCut = new QShortcut(QKeySequence(Qt::Key_Return), this);

    connect(m_findSourceButton, &QPushButton::clicked, this, &AdvancedInputWidget::findSourceButtonPressed);
    connect(m_topicNameComboBox, &QComboBox::currentTextChanged, this, [this] (const QString& text) {
        writeParameterToSettings(m_parameters.topicName, text, m_settings);
    });
    connect(findTargetButton, &QPushButton::clicked, this, &AdvancedInputWidget::findTargetButtonPressed);
    connect(m_dialogButtonBox, &QDialogButtonBox::accepted, this, &AdvancedInputWidget::okButtonPressed);
    connect(okShortCut, &QShortcut::activated, this, &AdvancedInputWidget::okButtonPressed);

    setLowDiskSpaceWidgetVisibility(m_targetLineEdit->text());
}


void
AdvancedInputWidget::findSourceButtonPressed()
{
    const auto bagDirectory = QFileDialog::getExistingDirectory(this, "Open Source Bag File", "", QFileDialog::ShowDirsOnly);
    if (bagDirectory.isEmpty()) {
        return;
    }
    // Automatically fill with available topic names
    QString topicType;
    QString autoTargetDir;
    switch (m_outputFormat) {
    case OUTPUT_VIDEO:
        topicType = "sensor_msgs/msg/Image";
        autoTargetDir = "/bag_video." + m_videoFormat;
        break;
    case OUTPUT_IMAGES:
        topicType = "sensor_msgs/msg/Image";
        autoTargetDir = "/image_files";
        break;
    case OUTPUT_PCDS:
        topicType = "sensor_msgs/msg/PointCloud2";
        autoTargetDir = "/pcd_files";
        break;
    default:
        break;
    }

    if (const auto containsTopics = Utils::UI::fillComboBoxWithTopics(m_topicNameComboBox, bagDirectory, topicType); !containsTopics) {
        Utils::UI::createCriticalMessageBox("Topic not found!", "The bag file does not contain any corresponding topics!");
        return;
    }

    m_sourceLineEdit->setText(bagDirectory);
    writeParameterToSettings(m_parameters.sourceDirectory, bagDirectory, m_settings);

    QDir bagDirectoryDir(bagDirectory);
    // Automatically fill up the target dir if there is no already existing name
    bagDirectoryDir.cdUp();
    if (const auto autoTargetDirectory = bagDirectoryDir.path() + autoTargetDir; !std::filesystem::exists(autoTargetDirectory.toStdString())) {
        m_targetLineEdit->setText(autoTargetDirectory);

        writeParameterToSettings(m_parameters.targetDirectory, autoTargetDirectory, m_settings);
        setLowDiskSpaceWidgetVisibility(m_targetLineEdit->text());
    }

    enableOkButton(!m_parameters.sourceDirectory.isEmpty() &&
                   !m_parameters.topicName.isEmpty() && !m_parameters.targetDirectory.isEmpty());
}


void
AdvancedInputWidget::findTargetButtonPressed()
{
    QString fileName;
    switch (m_outputFormat) {
    case OUTPUT_VIDEO:
        fileName = QFileDialog::getSaveFileName(this, "Save Video", "", m_videoFormat + " files (*." + m_videoFormat + ")");
        break;
    case OUTPUT_IMAGES:
    case OUTPUT_PCDS:
        fileName = QFileDialog::getExistingDirectory(this, "Save Files", "", QFileDialog::ShowDirsOnly);
        break;
    case OUTPUT_BAG:
        fileName = QFileDialog::getSaveFileName(this, "Save Bag");
        break;
    }
    if (fileName.isEmpty()) {
        return;
    }

    writeParameterToSettings(m_parameters.targetDirectory, fileName, m_settings);
    m_targetLineEdit->setText(fileName);

    enableOkButton(!m_parameters.sourceDirectory.isEmpty() &&
                   !m_parameters.topicName.isEmpty() && !m_parameters.targetDirectory.isEmpty());
    setLowDiskSpaceWidgetVisibility(m_targetLineEdit->text());
}


void
AdvancedInputWidget::okButtonPressed() const
{
    if (!m_okButton->isEnabled()) {
        return;
    }

    if (const auto sufficientSpace = showLowDiskSpaceMessageBox(); !sufficientSpace) {
        return;
    }
    if (!Utils::ROS::doesDirectoryContainBagFile(m_parameters.sourceDirectory)) {
        Utils::UI::createCriticalMessageBox("Invalid bag file!", "The source bag file seems to be invalid or broken!");
        return;
    }
    if (!Utils::UI::continueForExistingTarget(m_parameters.targetDirectory,
                                              m_outputFormat == OUTPUT_VIDEO ? "Video" : "Directory",
                                              m_outputFormat == OUTPUT_VIDEO ? "video" : "directory")) {
        return;
    }

    emit okPressed();
}
