#include "BasicBagWidget.hpp"

#include "MessageCountWidget.hpp"
#include "UtilsROS.hpp"
#include "UtilsUI.hpp"

#include <QCheckBox>
#include <QDialogButtonBox>
#include <QFileDialog>
#include <QFormLayout>
#include <QLabel>
#include <QMessageBox>
#include <QPushButton>
#include <QToolButton>
#include <QTreeWidget>
#include <QVBoxLayout>

#include <filesystem>

BasicBagWidget::BasicBagWidget(Parameters::AdvancedParameters& parameters,
                               const QString& titleText, const QString& iconText, const QString& settingsIdentifierText,
                               QWidget *parent) :
    BasicInputWidget(titleText, iconText, parent),
    m_parameters(parameters), m_settings(parameters, settingsIdentifierText)
{
    if (!std::filesystem::exists(m_parameters.sourceDirectory.toStdString()) ||
        !Utils::ROS::doesDirectoryContainBagFile(m_parameters.sourceDirectory)) {
        m_parameters.sourceDirectory = QString();
        writeParameterToSettings(m_parameters.sourceDirectory, QString(), m_settings);
    }
    m_sourceLineEdit->setText(m_parameters.sourceDirectory);

    m_treeWidget = new QTreeWidget;
    m_treeWidget->setVisible(false);
    m_treeWidget->setColumnCount(3);
    m_treeWidget->headerItem()->setText(COL_CHECKBOXES, "");
    m_treeWidget->headerItem()->setText(COL_TOPIC_NAME, "Topic Name:");
    m_treeWidget->headerItem()->setText(COL_TOPIC_TYPE, "Topic Type:");
    m_treeWidget->setRootIsDecorated(false);

    m_targetLineEdit = new QLineEdit(m_parameters.targetDirectory);
    auto* const targetPushButton = new QToolButton;
    auto* const targetLineEditLayout = Utils::UI::createLineEditButtonLayout(m_targetLineEdit, targetPushButton);

    auto* const targetFormLayout = new QFormLayout;
    targetFormLayout->addRow("Target Location:", targetLineEditLayout);
    targetFormLayout->setContentsMargins(0, 0, 0, 0);

    m_targetBagNameWidget = new QWidget;
    m_targetBagNameWidget->setLayout(targetFormLayout);
    m_targetBagNameWidget->setVisible(false);

    m_okButton->setEnabled(true);
    m_okButton->setVisible(false);

    connect(m_treeWidget, &QTreeWidget::itemChanged, this, &BasicBagWidget::itemCheckStateChanged);
    connect(targetPushButton, &QPushButton::clicked, this, &BasicBagWidget::targetPushButtonPressed);
}


void
BasicBagWidget::itemCheckStateChanged(QTreeWidgetItem* item, int /* column */)
{
    // Disable item widgets, this improves distinction between enabed and disabled topics
    m_treeWidget->itemWidget(item, COL_TOPIC_NAME)->setEnabled(item->checkState(COL_CHECKBOXES) == Qt::Checked ? true : false);
    m_treeWidget->itemWidget(item, COL_TOPIC_TYPE)->setEnabled(item->checkState(COL_CHECKBOXES) == Qt::Checked ? true : false);
}


void
BasicBagWidget::targetPushButtonPressed()
{
    const auto fileName = QFileDialog::getSaveFileName(this, "Target Bag File", "", "");
    if (fileName.isEmpty()) {
        return;
    }

    m_targetLineEdit->setText(fileName);
    writeParameterToSettings(m_parameters.targetDirectory, fileName, m_settings);
}


bool
BasicBagWidget::areIOParametersValid(int topicSize, int topicSizeWithOutDuplicates,
                                     const QString& secondSourceParameter)
{
    QVector<QString> sourceParameters = { m_parameters.sourceDirectory };
    if (!secondSourceParameter.isEmpty()) {
        sourceParameters.push_back(secondSourceParameter);
    }

    const auto containsBagFile = [] (const auto& directory) {
        return !Utils::ROS::doesDirectoryContainBagFile(directory);
    };
    // Critical errors first, then messageboxes allowing to continue
    if (std::any_of(sourceParameters.begin(), sourceParameters.end(), containsBagFile)) {
        Utils::UI::createCriticalMessageBox(sourceParameters.size() == 1 ? "Invalid bag file!" : "Invalid bag files!",
                                            sourceParameters.size() == 1 ? "The source bag file seems to be invalid!" : "A source bag file seems to be invalid!");
        return false;
    }
    if (std::any_of(sourceParameters.begin(), sourceParameters.end(), [this] (const auto& parameter) {
        return parameter == m_parameters.targetDirectory;
    })) {
        auto *const msgBox = new QMessageBox(QMessageBox::Critical, "Equal files!",
                                             "Source and target dir have the same path. Please enter a different name for the target file!",
                                             QMessageBox::Ok);
        msgBox->exec();
        return false;
    }
    if (m_targetLineEdit->text().isEmpty()) {
        auto *const msgBox = new QMessageBox(QMessageBox::Critical, "No target specified!", "Please make sure that a target file has been entered!",
                                             QMessageBox::Ok);
        msgBox->exec();
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
        if (const auto ret = msgBox->exec(); ret == QMessageBox::No) {
            return false;
        }
    }
    return true;
}
