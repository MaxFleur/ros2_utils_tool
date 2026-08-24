#include "AdvancedBagWidget.hpp"

#include "BagTreeWidget.hpp"
#include "UtilsROS.hpp"
#include "UtilsUI.hpp"

#include <QCheckBox>
#include <QComboBox>
#include <QFormLayout>
#include <QLabel>
#include <QMessageBox>
#include <QPushButton>
#include <QTreeWidget>

#include <filesystem>

AdvancedBagWidget::AdvancedBagWidget(Parameters::DeleteSourceParameters& parameters,
                                     const QString& titleText, const QString& iconText, const QString& settingsIdentifierText, const int outputFormat,
                                     QWidget *parent) :
    AdvancedInputWidget(parameters, titleText, iconText, "Source Bag:", "Target Bag:", settingsIdentifierText, outputFormat, parent),
    m_parameters(parameters), m_settings(parameters, settingsIdentifierText)
{
    // Take the source and target form layout for manipulation
    m_controlsLayout->takeAt(4);
    // Reuse the target layout for a new form layout
    const auto targetLocationRow = m_basicOptionsFormLayout->takeRow(1);

    auto* const findTargetLayout = new QFormLayout;
    findTargetLayout->addRow(targetLocationRow.labelItem->widget(), targetLocationRow.fieldItem->layout());

    m_findTargetWidget = new QWidget;
    m_findTargetWidget->setLayout(findTargetLayout);
    m_findTargetWidget->setVisible(false);

    if (!std::filesystem::exists(m_parameters.sourceDirectory.toStdString()) ||
        !Utils::ROS::doesDirectoryContainBagFile(m_parameters.sourceDirectory)) {
        m_parameters.sourceDirectory = QString();
        writeParameterToSettings(m_parameters.sourceDirectory, QString(), m_settings);
    }

    m_deleteSourceCheckBox = new QCheckBox("Delete Source Bag File(s) after Completion");
    m_deleteSourceCheckBox->setTristate(false);
    m_deleteSourceCheckBox->setChecked(m_parameters.deleteSource);

    m_compressionLabel = new QLabel("Compression Mode:");
    m_compressionLabel->setVisible(false);

    m_compressionModeComboBox = new QComboBox;
    m_compressionModeComboBox->addItem("None");
    m_compressionModeComboBox->addItem("By File");
    m_compressionModeComboBox->addItem("Per Message");
    m_compressionModeComboBox->setCurrentIndex(COMPRESSION_NONE);
    m_compressionModeComboBox->setVisible(false);

    m_compressionLayout = new QHBoxLayout;
    m_compressionLayout->addWidget(m_compressionLabel);
    m_compressionLayout->addWidget(m_compressionModeComboBox);
    m_compressionLayout->addStretch();

    m_treeWidget = new BagTreeWidget;

    m_controlsLayout->addLayout(m_basicOptionsFormLayout);
    m_controlsLayout->addSpacing(10);

    m_okButton->setEnabled(true);
    m_okButton->setVisible(false);

    connect(m_treeWidget, &QTreeWidget::itemChanged, this, &AdvancedBagWidget::itemCheckStateChanged);
    connect(m_deleteSourceCheckBox, &QCheckBox::stateChanged, this, [this] (int state) {
        writeParameterToSettings(m_parameters.deleteSource, state == Qt::Checked, m_settings);
    });
}


bool
AdvancedBagWidget::areIOParametersValid(int topicSize, int topicSizeWithOutDuplicates,
                                        const QString& secondSourceParameter) const
{
    QVector<QString> sourceParameters = { m_parameters.sourceDirectory };
    if (!secondSourceParameter.isEmpty()) {
        sourceParameters.push_back(secondSourceParameter);
    }

    const auto containsBagFile = [] (const auto& directory) {
        return !Utils::ROS::doesDirectoryContainBagFile(directory);
    };
    // Critical errors first, then messageboxes allowing to continue
    if (std::ranges::any_of(sourceParameters, containsBagFile)) {
        Utils::UI::createCriticalMessageBox(sourceParameters.size() == 1 ? "Invalid bag file!" : "Invalid bag files!",
                                            sourceParameters.size() == 1 ? "The source bag file seems to be invalid!" : "A source bag file seems to be invalid!");
        return false;
    }

    if (std::ranges::any_of(sourceParameters, [this] (const auto& parameter) {
        return parameter == m_parameters.targetDirectory;
    })) {
        auto *const msgBox = new QMessageBox(QMessageBox::Critical, "Equal files!",
                                             "Source and target dir have the same path. Please enter a different name for the target file!",
                                             QMessageBox::Ok);
        msgBox->setAttribute(Qt::WA_DeleteOnClose);
        msgBox->exec();

        return false;
    }

    if (m_targetLineEdit->text().isEmpty()) {
        auto *const msgBox = new QMessageBox(QMessageBox::Critical, "No target specified!", "Please make sure that a target file has been entered!",
                                             QMessageBox::Ok);
        msgBox->setAttribute(Qt::WA_DeleteOnClose);
        msgBox->exec();

        return false;
    }

    if (const auto sufficientSpace = showLowDiskSpaceMessageBox(); !sufficientSpace) {
        return false;
    }
    if (!Utils::UI::continueForExistingTarget(m_parameters.targetDirectory, "Bag file", "bag file")) {
        return false;
    }
    if (topicSize != topicSizeWithOutDuplicates) {
        auto *const msgBox = new QMessageBox(QMessageBox::Warning, "Duplicate topic names!",
                                             "Duplicate topic names were selected, which is not allowed in ROS bag files. These would be merged into one topic.\n"
                                             "Are you sure you want to continue? ",
                                             QMessageBox::Yes | QMessageBox::No);
        msgBox->setAttribute(Qt::WA_DeleteOnClose);

        if (const auto ret = msgBox->exec(); ret == QMessageBox::No) {
            return false;
        }
    }
    return true;
}
