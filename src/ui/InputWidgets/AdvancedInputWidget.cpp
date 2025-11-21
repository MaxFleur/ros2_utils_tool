#include "AdvancedInputWidget.hpp"

#include "LowDiskSpaceWidget.hpp"
#include "UtilsROS.hpp"
#include "UtilsUI.hpp"

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

    m_sourceLineEdit->setText(bagDirectory);
    writeParameterToSettings(m_parameters.sourceDirectory, bagDirectory, m_settings);
    fillTargetLineEdit();

    enableOkButton(!m_parameters.sourceDirectory.isEmpty() &&
                   !m_parameters.topicName.isEmpty() && !m_parameters.targetDirectory.isEmpty());
}


void
AdvancedInputWidget::findTargetButtonPressed()
{
    QString fileName;
    switch (m_outputFormat) {
    case OUTPUT_VIDEO:
        fileName = QFileDialog::getSaveFileName(this, "Save Video", "", m_fileFormat + " files (*." + m_fileFormat + ")");
        break;
    case OUTPUT_IMAGES:
    case OUTPUT_PCDS:
        fileName = QFileDialog::getExistingDirectory(this, "Save Files", "", QFileDialog::ShowDirsOnly);
        break;
    case OUTPUT_TF_TO_FILE:
        fileName = QFileDialog::getSaveFileName(this, "Save File", "", m_fileFormat + " files (*." + m_fileFormat + ")");
        break;
    case OUTPUT_BAG:
    case OUTPUT_BAG_EDITED:
    case OUTPUT_BAG_MERGED:
    case OUTPUT_BAG_COMPRESSED:
    case OUTPUT_BAG_DECOMPRESSED:
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


void
AdvancedInputWidget::fillTargetLineEdit()
{
    QString autoTargetDir;

    switch (m_outputFormat) {
    case OUTPUT_VIDEO:
        autoTargetDir = "/bag_video." + m_fileFormat;
        break;
    case OUTPUT_IMAGES:
        autoTargetDir = "/image_files";
        break;
    case OUTPUT_PCDS:
        autoTargetDir = "/pcd_files";
        break;
    case OUTPUT_TF_TO_FILE:
        autoTargetDir = "/bag_transforms." + m_fileFormat;
        break;
    case OUTPUT_BAG_EDITED:
        autoTargetDir = "/edited_bag";
        break;
    case OUTPUT_BAG_MERGED:
        autoTargetDir = "/merged_bag";
        break;
    case OUTPUT_BAG_COMPRESSED:
        autoTargetDir = "/compressed_bag";
        break;
    case OUTPUT_BAG_DECOMPRESSED:
        autoTargetDir = "/decompressed_bag";
        break;
    default:
        break;
    }

    QDir bagDirectoryDir(m_sourceLineEdit->text());
    // Automatically fill up the target dir if there is no already existing name
    bagDirectoryDir.cdUp();
    if (const auto autoTargetDirectory = bagDirectoryDir.path() + autoTargetDir; !std::filesystem::exists(autoTargetDirectory.toStdString())) {
        m_targetLineEdit->setText(autoTargetDirectory);

        writeParameterToSettings(m_parameters.targetDirectory, autoTargetDirectory, m_settings);
        setLowDiskSpaceWidgetVisibility(m_targetLineEdit->text());
    }
}
