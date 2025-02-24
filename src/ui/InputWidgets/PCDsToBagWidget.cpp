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
#include <QShortcut>
#include <QToolButton>
#include <QVBoxLayout>

#include <filesystem>

PCDsToBagWidget::PCDsToBagWidget(Parameters::AdvancedParameters& parameters,
                                 bool usePredefinedTopicName, bool checkROS2NameConform, QWidget *parent) :
    AdvancedInputWidget(parameters, "PCD Files to Bag", ":/icons/pcd_to_bag", "pcd_to_bag", OUTPUT_BAG, parent),
    m_parameters(parameters), m_settings(parameters, "pcd_to_bag"),
    m_checkROS2NameConform(checkROS2NameConform)
{
    m_targetLineEdit->setToolTip("The directory where the bag should be stored.");

    auto* const bagLocationButton = new QToolButton;
    auto* const storeBagLayout = Utils::UI::createLineEditButtonLayout(m_targetLineEdit, bagLocationButton);

    auto* const topicNameLineEdit = new QLineEdit(m_parameters.topicName);
    topicNameLineEdit->setToolTip("The topic name in the target bag file.");
    // Use a predefined name if set in the settings
    if (usePredefinedTopicName && m_parameters.topicName.isEmpty()) {
        topicNameLineEdit->setText("/topic_point_cloud");
    }

    auto* const formLayout = new QFormLayout;
    formLayout->addRow("PCD Files:", m_findSourceLayout);
    formLayout->addRow("Bag Location:", storeBagLayout);
    formLayout->addRow("Topic Name:", topicNameLineEdit);

    auto* const controlsLayout = new QVBoxLayout;
    controlsLayout->addStretch();
    controlsLayout->addWidget(m_headerPixmapLabel);
    controlsLayout->addWidget(m_headerLabel);
    controlsLayout->addSpacing(40);
    controlsLayout->addLayout(formLayout);
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
    // Generally, enable ok only if we have a source and target dir and an existing topic name
    enableOkButton(!m_parameters.sourceDirectory.isEmpty() && !m_parameters.targetDirectory.isEmpty() && !m_parameters.topicName.isEmpty());

    connect(bagLocationButton, &QPushButton::clicked, this, &PCDsToBagWidget::targetLocationButtonPressed);
    connect(topicNameLineEdit, &QLineEdit::textChanged, this, [this, topicNameLineEdit] {
        writeParameterToSettings(m_parameters.topicName, topicNameLineEdit->text(), m_settings);
        enableOkButton(!m_parameters.sourceDirectory.isEmpty() && !m_parameters.targetDirectory.isEmpty() && !m_parameters.topicName.isEmpty());
    });
    connect(okShortCut, &QShortcut::activated, this, &PCDsToBagWidget::okButtonPressed);
}


void
PCDsToBagWidget::searchButtonPressed()
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
    }

    enableOkButton(!m_parameters.sourceDirectory.isEmpty() && !m_parameters.targetDirectory.isEmpty() && !m_parameters.topicName.isEmpty());
}


void
PCDsToBagWidget::okButtonPressed()
{
    if (!m_okButton->isEnabled()) {
        return;
    }

    if (m_checkROS2NameConform && !Utils::ROS::isNameROS2Conform(m_parameters.topicName)) {
        if (const auto returnValue = Utils::UI::continueWithInvalidROS2Names(); !returnValue) {
            return;
        }
    }
    if (!Utils::UI::continueForExistingTarget(m_parameters.targetDirectory, "Bag file", "bag file")) {
        return;
    }

    emit okPressed();
}
