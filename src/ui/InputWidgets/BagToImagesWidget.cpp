#include "BagToImagesWidget.hpp"

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

BagToImagesWidget::BagToImagesWidget(Utils::UI::ImageInputParameters& parameters, QWidget *parent) :
    BasicInputWidget("Write Images from Bag", ":/icons/bag_to_images", parent),
    m_parameters(parameters), m_settings(parameters, "bag_to_images")
{
    m_sourceLineEdit->setText(parameters.sourceDirectory);
    m_sourceLineEdit->setToolTip("The directory of the ROSBag source file.");

    m_topicNameComboBox = new QComboBox;
    m_topicNameComboBox->setMinimumWidth(200);
    m_topicNameComboBox->setToolTip("The ROSBag topic of the video file.\nIf the Bag contains multiple video topics, you can choose one of them.");

    if (!m_parameters.sourceDirectory.isEmpty()) {
        Utils::UI::fillComboBoxWithTopics(m_topicNameComboBox, m_parameters.sourceDirectory);

        if (!m_parameters.topicName.isEmpty()) {
            m_topicNameComboBox->setCurrentText(m_parameters.topicName);
        }
    }

    m_imagesNameLineEdit = new QLineEdit(m_parameters.targetDirectory);
    m_imagesNameLineEdit->setToolTip("The directory where the images should be stored.");

    auto* const imagesLocationButton = new QToolButton;
    auto* const searchImagesPathLayout = Utils::UI::createLineEditButtonLayout(m_imagesNameLineEdit, imagesLocationButton);

    auto* const formatComboBox = new QComboBox;
    formatComboBox->addItem("jpg", 0);
    formatComboBox->addItem("png", 1);
    formatComboBox->addItem("bmp", 2);
    formatComboBox->setToolTip("The format of the written images.");
    formatComboBox->setCurrentText(m_parameters.format);

    auto* const basicOptionsFormLayout = new QFormLayout;
    basicOptionsFormLayout->addRow("Bag File:", m_findSourceLayout);
    basicOptionsFormLayout->addRow("Topic Name:", m_topicNameComboBox);
    basicOptionsFormLayout->addRow("Images Location:", searchImagesPathLayout);
    basicOptionsFormLayout->addRow("Format:", formatComboBox);

    auto* const advancedOptionsCheckBox = new QCheckBox;
    advancedOptionsCheckBox->setChecked(m_parameters.showAdvancedOptions ? Qt::Checked : Qt::Unchecked);
    advancedOptionsCheckBox->setText("Show Advanced Options");

    auto* const switchRedBlueCheckBox = Utils::UI::createCheckBox("Switch the video's red and blue values.", m_parameters.exchangeRedBlueValues);
    auto* const useBWCheckBox = Utils::UI::createCheckBox("If the images should be colorless or not.", m_parameters.useBWImages);

    m_advancedOptionsFormLayout = new QFormLayout;
    m_advancedOptionsFormLayout->addRow("Switch Red and Blue Values:", switchRedBlueCheckBox);
    m_advancedOptionsFormLayout->addRow("Colorless Images:", useBWCheckBox);

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

    adjustWidgetsToChangedFormat(m_parameters.format);

    auto* const mainLayout = new QVBoxLayout;
    mainLayout->addLayout(controlsSqueezedLayout);
    mainLayout->addLayout(m_buttonLayout);
    setLayout(mainLayout);

    auto* const okShortCut = new QShortcut(QKeySequence(Qt::Key_Return), this);
    // Generally, only enable this if the source bag, topic name and target dir line edit contain text
    enableOkButton(!m_parameters.sourceDirectory.isEmpty() &&
                   !m_parameters.topicName.isEmpty() && !m_parameters.targetDirectory.isEmpty());


    connect(m_findSourceButton, &QPushButton::clicked, this, &BagToImagesWidget::searchButtonPressed);
    connect(m_topicNameComboBox, &QComboBox::currentTextChanged, this, [this] (const QString& text) {
        writeParameterToSettings(m_parameters.topicName, text, m_settings);
    });
    connect(imagesLocationButton, &QPushButton::clicked, this, &BagToImagesWidget::imagesLocationButtonPressed);
    connect(formatComboBox, &QComboBox::currentTextChanged, this, &BagToImagesWidget::adjustWidgetsToChangedFormat);
    connect(advancedOptionsCheckBox, &QCheckBox::stateChanged, this, [this, advancedOptionsWidget] (int state) {
        m_parameters.showAdvancedOptions = state == Qt::Checked;
        advancedOptionsWidget->setVisible(state == Qt::Checked);
    });
    connect(switchRedBlueCheckBox, &QCheckBox::stateChanged, this, [this] (int state) {
        writeParameterToSettings(m_parameters.exchangeRedBlueValues, state == Qt::Checked, m_settings);
    });
    connect(useBWCheckBox, &QCheckBox::stateChanged, this, [this] (int state) {
        writeParameterToSettings(m_parameters.useBWImages, state == Qt::Checked, m_settings);
    });
    connect(m_dialogButtonBox, &QDialogButtonBox::accepted, this, &BagToImagesWidget::okButtonPressed);
    connect(okShortCut, &QShortcut::activated, this, &BagToImagesWidget::okButtonPressed);
}


void
BagToImagesWidget::searchButtonPressed()
{
    const auto bagDirectory = QFileDialog::getExistingDirectory(this, "Open ROSBag", "",
                                                                QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
    if (bagDirectory.isEmpty()) {
        return;
    }
    // Automatically fill with available topic names
    if (const auto containsVideoTopics = Utils::UI::fillComboBoxWithTopics(m_topicNameComboBox, bagDirectory); !containsVideoTopics) {
        Utils::UI::createCriticalMessageBox("Topic not found!", "The bag file does not contain any image/video topics!");
        return;
    }

    m_sourceLineEdit->setText(bagDirectory);
    writeParameterToSettings(m_parameters.sourceDirectory, bagDirectory, m_settings);

    QDir bagDirectoryDir(bagDirectory);
    // Automatically fill up the image target dir if there isn't an already existing name
    bagDirectoryDir.cdUp();
    if (const auto autoImageDirectory = bagDirectoryDir.path() + "/bag_images"; !std::filesystem::exists(autoImageDirectory.toStdString())) {
        m_imagesNameLineEdit->setText(autoImageDirectory);
        writeParameterToSettings(m_parameters.targetDirectory, autoImageDirectory, m_settings);
    }

    enableOkButton(!m_parameters.sourceDirectory.isEmpty() &&
                   !m_parameters.topicName.isEmpty() && !m_parameters.targetDirectory.isEmpty());
}


void
BagToImagesWidget::imagesLocationButtonPressed()
{
    const auto fileName = QFileDialog::getExistingDirectory(this, "Save Images", "", QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
    if (fileName.isEmpty()) {
        return;
    }

    m_fileDialogOpened = true;
    writeParameterToSettings(m_parameters.targetDirectory, fileName, m_settings);
    m_imagesNameLineEdit->setText(fileName);
    enableOkButton(!m_parameters.sourceDirectory.isEmpty() &&
                   !m_parameters.topicName.isEmpty() && !m_parameters.targetDirectory.isEmpty());
}


// Some parameters also change if the images format is changed, so we need to update the UI accordingly
void
BagToImagesWidget::adjustWidgetsToChangedFormat(const QString& text)
{
    writeParameterToSettings(m_parameters.format, text, m_settings);

    if (m_optimizeOrBilevelCheckBox && m_qualitySlider) {
        m_advancedOptionsFormLayout->removeRow(m_optimizeOrBilevelCheckBox);
        m_advancedOptionsFormLayout->removeRow(m_qualitySlider);
    }
    if (text == "bmp") {
        return;
    }

    m_qualitySlider = new QSlider(Qt::Horizontal);
    m_qualitySlider->setRange(0, 9);
    m_qualitySlider->setTickInterval(1);
    m_qualitySlider->setValue(m_parameters.quality);
    m_qualitySlider->setTickPosition(QSlider::TicksBelow);
    m_qualitySlider->setToolTip(text == "jpg" ? "Image quality. A higher quality will increase the image size."
                                              : "Higher compression will result in smaller size, but increase writing time.");

    m_optimizeOrBilevelCheckBox = new QCheckBox;
    auto& memberVal = text == "jpg" ? m_parameters.jpgOptimize : m_parameters.pngBilevel;
    m_optimizeOrBilevelCheckBox->setChecked(memberVal ? Qt::Checked : Qt::Unchecked);
    m_optimizeOrBilevelCheckBox->setToolTip(text == "jpg" ? "Optimize the stored file size." : "Save as an image containing only black and white pixels.");

    m_advancedOptionsFormLayout->insertRow(0, text == "jpg" ? "Quality:" : "Level of Compression:", m_qualitySlider);
    m_advancedOptionsFormLayout->insertRow(1, text == "jpg" ? "Optimize Size" : "Binary Image", m_optimizeOrBilevelCheckBox);

    connect(m_qualitySlider, &QSlider::valueChanged, this, [this] (int value) {
        writeParameterToSettings(m_parameters.quality, value, m_settings);
    });
    connect(m_optimizeOrBilevelCheckBox, &QCheckBox::stateChanged, this, [this] (int state) {
        m_parameters.format == "jpg" ?
        writeParameterToSettings(m_parameters.jpgOptimize, state == Qt::Checked, m_settings) :
        writeParameterToSettings(m_parameters.pngBilevel, state == Qt::Checked, m_settings);
    });
}


void
BagToImagesWidget::okButtonPressed()
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
