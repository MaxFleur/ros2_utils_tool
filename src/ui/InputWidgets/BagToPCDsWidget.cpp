#include "BagToPCDsWidget.hpp"

#include "UtilsROS.hpp"

#include <QCheckBox>
#include <QComboBox>
#include <QDialogButtonBox>
#include <QFileDialog>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QSlider>
#include <QShortcut>
#include <QToolButton>
#include <QVBoxLayout>

#include <filesystem>

BagToPCDsWidget::BagToPCDsWidget(Utils::UI::AdvancedInputParameters& parameters, QWidget *parent) :
    BasicInputWidget("Write PCD Files from Bag", ":/icons/bag_to_pcd", parent),
    m_parameters(parameters), m_settings(parameters, "bag_to_pcds")
{
    m_sourceLineEdit->setText(parameters.sourceDirectory);
    m_sourceLineEdit->setToolTip("The directory of the ROSBag source file.");

    m_topicNameComboBox = new QComboBox;
    m_topicNameComboBox->setMinimumWidth(200);
    m_topicNameComboBox->setToolTip("The point cloud bag topic.\nIf the Bag contains multiple point cloud topics, you can choose one of them.");

    if (!m_parameters.sourceDirectory.isEmpty()) {
        Utils::UI::fillComboBoxWithTopics(m_topicNameComboBox, m_parameters.sourceDirectory, "sensor_msgs/msg/PointCloud2");

        if (!m_parameters.topicName.isEmpty()) {
            m_topicNameComboBox->setCurrentText(m_parameters.topicName);
        }
    }

    m_pcdsNameLineEdit = new QLineEdit(m_parameters.targetDirectory);
    m_pcdsNameLineEdit->setToolTip("The directory where the images should be stored.");

    auto* const pcdsLocationButton = new QToolButton;
    auto* const searchImagesPathLayout = Utils::UI::createLineEditButtonLayout(m_pcdsNameLineEdit, pcdsLocationButton);

    auto* const basicOptionsFormLayout = new QFormLayout;
    basicOptionsFormLayout->addRow("Bag File:", m_findSourceLayout);
    basicOptionsFormLayout->addRow("Topic Name:", m_topicNameComboBox);
    basicOptionsFormLayout->addRow("Images Location:", searchImagesPathLayout);

    auto* const controlsLayout = new QVBoxLayout;
    controlsLayout->addStretch();
    controlsLayout->addWidget(m_headerPixmapLabel);
    controlsLayout->addWidget(m_headerLabel);
    controlsLayout->addSpacing(40);
    controlsLayout->addLayout(basicOptionsFormLayout);
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
    // Generally, only enable this if the source bag, topic name and target dir line edit contain text
    enableOkButton(!m_parameters.sourceDirectory.isEmpty() &&
                   !m_parameters.topicName.isEmpty() && !m_parameters.targetDirectory.isEmpty());


    connect(m_findSourceButton, &QPushButton::clicked, this, &BagToPCDsWidget::searchButtonPressed);
    connect(m_topicNameComboBox, &QComboBox::currentTextChanged, this, [this] (const QString& text) {
        writeParameterToSettings(m_parameters.topicName, text, m_settings);
    });
    connect(pcdsLocationButton, &QPushButton::clicked, this, &BagToPCDsWidget::pcdsLocationButtonPressed);
    connect(m_dialogButtonBox, &QDialogButtonBox::accepted, this, &BagToPCDsWidget::okButtonPressed);
    connect(okShortCut, &QShortcut::activated, this, &BagToPCDsWidget::okButtonPressed);
}


void
BagToPCDsWidget::searchButtonPressed()
{
    const auto bagDirectory = QFileDialog::getExistingDirectory(this, "Open ROSBag", "",
                                                                QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
    if (bagDirectory.isEmpty()) {
        return;
    }
    // Automatically fill with available topic names
    if (const auto containsVideoTopics = Utils::UI::fillComboBoxWithTopics(m_topicNameComboBox, bagDirectory, "sensor_msgs/msg/PointCloud2");
        !containsVideoTopics) {
        Utils::UI::createCriticalMessageBox("Topic not found!", "The bag file does not contain any point cloud topics!");
        return;
    }

    m_sourceLineEdit->setText(bagDirectory);
    writeParameterToSettings(m_parameters.sourceDirectory, bagDirectory, m_settings);

    QDir bagDirectoryDir(bagDirectory);
    // Automatically fill up the target dir if there is no already existing name
    bagDirectoryDir.cdUp();
    if (const auto autoImageDirectory = bagDirectoryDir.path() + "/pcd_files"; !std::filesystem::exists(autoImageDirectory.toStdString())) {
        m_pcdsNameLineEdit->setText(autoImageDirectory);
        writeParameterToSettings(m_parameters.targetDirectory, autoImageDirectory, m_settings);
    }

    enableOkButton(!m_parameters.sourceDirectory.isEmpty() &&
                   !m_parameters.topicName.isEmpty() && !m_parameters.targetDirectory.isEmpty());
}


void
BagToPCDsWidget::pcdsLocationButtonPressed()
{
    const auto fileName = QFileDialog::getExistingDirectory(this, "Save PCD Files", "", QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
    if (fileName.isEmpty()) {
        return;
    }

    writeParameterToSettings(m_parameters.targetDirectory, fileName, m_settings);
    m_pcdsNameLineEdit->setText(fileName);
    enableOkButton(!m_parameters.sourceDirectory.isEmpty() &&
                   !m_parameters.topicName.isEmpty() && !m_parameters.targetDirectory.isEmpty());
}


void
BagToPCDsWidget::okButtonPressed()
{
    if (!m_okButton->isEnabled()) {
        return;
    }

    if (!Utils::ROS::doesDirectoryContainBagFile(m_parameters.sourceDirectory)) {
        Utils::UI::createCriticalMessageBox("Invalid bag file!", "The source bag file seems to be invalid or broken!");
        return;
    }
    if (!Utils::UI::continueForExistingTarget(m_parameters.targetDirectory, "Directory", "directory")) {
        return;
    }

    emit okPressed();
}
