#include "PCDsToBagWidget.hpp"

#include "UtilsROS.hpp"
#include "UtilsUI.hpp"

#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QSpinBox>

#include <filesystem>

PCDsToBagWidget::PCDsToBagWidget(Parameters::PCDsToBagParameters& parameters,
                                 bool usePredefinedTopicName, bool warnROS2NameConvention, QWidget *parent) :
    TopicComboBoxWidget(parameters, "PCD Files to Bag", ":/icons/tools/pcd_to_bag", "Files Dir:", "Bag Location:", "pcd_to_bag", OUTPUT_BAG, parent),
    m_parameters(parameters), m_settings(parameters, "pcd_to_bag"),
    m_warnROS2NameConvention(warnROS2NameConvention)
{
    m_sourceLineEdit->setToolTip("The source pcd files directory.");
    m_targetLineEdit->setToolTip("The target bag file directory.");

    auto* const topicNameLineEdit = new QLineEdit(m_parameters.topicName);
    topicNameLineEdit->setToolTip("The topic name in the target bag file.");
    // Use a predefined name if set in the settings
    if (usePredefinedTopicName && m_parameters.topicName.isEmpty()) {
        topicNameLineEdit->setText("/topic_point_cloud");
        writeParameterToSettings(m_parameters.topicName, topicNameLineEdit->text(), m_settings);
    }

    auto* const rateSpinBox = new QSpinBox;
    rateSpinBox->setRange(1, 30);
    rateSpinBox->setValue(m_parameters.rate);
    rateSpinBox->setToolTip("The rate ('clouds per second') stored in the bag.");

    m_basicOptionsFormLayout->addRow("Topic Name:", topicNameLineEdit);
    m_basicOptionsFormLayout->addRow("Rate:", rateSpinBox);

    m_controlsLayout->addStretch();

    // Generally, enable ok only if we have a source and target dir and an existing topic name
    enableOkButton(!m_parameters.sourceDirectory.isEmpty() && !m_parameters.targetDirectory.isEmpty() && !m_parameters.topicName.isEmpty());

    connect(topicNameLineEdit, &QLineEdit::textChanged, this, [this, topicNameLineEdit] {
        writeParameterToSettings(m_parameters.topicName, topicNameLineEdit->text(), m_settings);
        enableOkButton(!m_parameters.sourceDirectory.isEmpty() && !m_parameters.targetDirectory.isEmpty() && !m_parameters.topicName.isEmpty());
    });
    connect(rateSpinBox, QOverload<int>::of(&QSpinBox::valueChanged), this, [this] (int value) {
        writeParameterToSettings(m_parameters.rate, value, m_settings);
    });
}


void
PCDsToBagWidget::findSourceButtonPressed()
{
    enableOkButton(false);

    const auto pcdDir = QFileDialog::getExistingDirectory(this, "Open PCD File Dir");
    if (pcdDir.isEmpty()) {
        return;
    }

    auto containsPCDFiles = false;
    for (auto const& entry : std::filesystem::directory_iterator(pcdDir.toStdString())) {
        if (entry.path().extension() == ".pcd") {
            containsPCDFiles = true;
            break;
        }
    }
    if (!containsPCDFiles) {
        auto *const msgBox = new QMessageBox(QMessageBox::Critical, "No pcd files!", "The directory does not contain pcd files!", QMessageBox::Ok);
        msgBox->exec();
        return;
    }

    writeParameterToSettings(m_parameters.sourceDirectory, pcdDir, m_settings);
    m_sourceLineEdit->setText(pcdDir);
    // Create bag file name automatically so the user has to put in less values
    QDir pcdFilesDirectory(pcdDir);
    pcdFilesDirectory.cdUp();
    if (const auto autoBagDirectory = pcdFilesDirectory.path() + "/bag_pcd_files"; !std::filesystem::exists(autoBagDirectory.toStdString())) {
        m_targetLineEdit->setText(autoBagDirectory);

        writeParameterToSettings(m_parameters.targetDirectory, autoBagDirectory, m_settings);
        setLowDiskSpaceWidgetVisibility(m_targetLineEdit->text());
    }

    enableOkButton(!m_parameters.sourceDirectory.isEmpty() && !m_parameters.targetDirectory.isEmpty() && !m_parameters.topicName.isEmpty());
}


void
PCDsToBagWidget::okButtonPressed() const
{
    if (!m_okButton->isEnabled()) {
        return;
    }

    if (const auto sufficientSpace = showLowDiskSpaceMessageBox(); !sufficientSpace) {
        return;
    }
    if (m_warnROS2NameConvention && !Utils::ROS::isNameROS2Conform(m_parameters.topicName)) {
        if (const auto returnValue = Utils::UI::continueWithInvalidROS2Names(); !returnValue) {
            return;
        }
    }
    if (!Utils::UI::continueForExistingTarget(m_parameters.targetDirectory, "Bag file", "bag file")) {
        return;
    }

    emit okPressed();
}
