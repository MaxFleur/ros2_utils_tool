#include "AdvancedInputWidget.hpp"

#include "UtilsROS.hpp"

#include <QComboBox>
#include <QDialogButtonBox>
#include <QFileDialog>
#include <QLineEdit>
#include <QPushButton>
#include <QShortcut>
#include <QToolButton>

#include <filesystem>

AdvancedInputWidget::AdvancedInputWidget(Utils::UI::AdvancedParameters& parameters, const QString& headerText,
                                         const QString& iconPath, const QString& settingsIdentifier,
                                         int outputFormat, QWidget *parent) :
    BasicInputWidget(headerText, iconPath, parent),
    m_parameters(parameters), m_settings(parameters, settingsIdentifier), m_outputFormat(outputFormat)
{
    m_sourceLineEdit->setText(parameters.sourceDirectory);
    m_sourceLineEdit->setToolTip("The directory of the source bag file.");

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

    auto* const okShortCut = new QShortcut(QKeySequence(Qt::Key_Return), this);

    connect(m_findSourceButton, &QPushButton::clicked, this, &AdvancedInputWidget::searchButtonPressed);
    connect(m_topicNameComboBox, &QComboBox::currentTextChanged, this, [this] (const QString& text) {
        writeParameterToSettings(m_parameters.topicName, text, m_settings);
    });
    connect(m_dialogButtonBox, &QDialogButtonBox::accepted, this, &AdvancedInputWidget::okButtonPressed);
    connect(okShortCut, &QShortcut::activated, this, &AdvancedInputWidget::okButtonPressed);
}


void
AdvancedInputWidget::searchButtonPressed()
{
    const auto bagDirectory = QFileDialog::getExistingDirectory(this, "Open ROSBag", "", QFileDialog::ShowDirsOnly);
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
    }

    enableOkButton(!m_parameters.sourceDirectory.isEmpty() &&
                   !m_parameters.topicName.isEmpty() && !m_parameters.targetDirectory.isEmpty());
}


void
AdvancedInputWidget::targetLocationButtonPressed()
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
}


void
AdvancedInputWidget::okButtonPressed()
{
    if (!m_okButton->isEnabled()) {
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
