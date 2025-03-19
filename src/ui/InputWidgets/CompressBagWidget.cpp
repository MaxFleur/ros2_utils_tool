#include "CompressBagWidget.hpp"

#include "UtilsROS.hpp"
#include "UtilsUI.hpp"

#include <QCheckBox>
#include <QComboBox>
#include <QFileDialog>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QVBoxLayout>

CompressBagWidget::CompressBagWidget(Parameters::CompressBagParameters& parameters, QWidget *parent) :
    BasicInputWidget("Compress Bag", ":/icons/compress_bag", parent),
    m_parameters(parameters), m_settings(parameters, "compress_bag")
{
    m_sourceLineEdit->setText(parameters.sourceDirectory);
    m_sourceLineEdit->setToolTip("The source bag file directory.");

    m_targetLineEdit = new QLineEdit(m_parameters.targetDirectory);
    m_targetLineEdit->setText(parameters.targetDirectory);
    m_targetLineEdit->setToolTip("The target compressed bag file directory.");

    auto* const targetLocationButton = new QToolButton;
    auto* const targetLocationLayout = Utils::UI::createLineEditButtonLayout(m_targetLineEdit, targetLocationButton);

    auto* const compressionFormatComboBox = new QComboBox;
    compressionFormatComboBox->addItem("By File");
    compressionFormatComboBox->addItem("Per Message");
    compressionFormatComboBox->setCurrentIndex(m_parameters.compressPerMessage);

    auto* const deleteSourceCheckBox = new QCheckBox;
    deleteSourceCheckBox->setCheckState(m_parameters.deleteSource ? Qt::Checked : Qt::Unchecked);

    auto* const formLayout = new QFormLayout;
    formLayout->addRow("Source Bag Location:", m_findSourceLayout);
    formLayout->addRow("Target Bag Location:", targetLocationLayout);
    formLayout->addRow("Compression Mode:", compressionFormatComboBox);
    formLayout->addRow("Delete Source:", deleteSourceCheckBox);

    auto* const controlsLayout = new QVBoxLayout;
    controlsLayout->addStretch();
    controlsLayout->addWidget(m_headerPixmapLabel);
    controlsLayout->addWidget(m_headerLabel);
    controlsLayout->addSpacing(40);
    controlsLayout->addLayout(formLayout);
    controlsLayout->addSpacing(20);
    controlsLayout->addStretch();

    auto* const controlsSqueezedLayout = new QHBoxLayout;
    controlsSqueezedLayout->addStretch();
    controlsSqueezedLayout->addLayout(controlsLayout);
    controlsSqueezedLayout->addStretch();

    m_okButton->setEnabled(true);

    auto* const mainLayout = new QVBoxLayout;
    mainLayout->addLayout(controlsSqueezedLayout);
    mainLayout->addLayout(m_buttonLayout);
    setLayout(mainLayout);

    connect(m_findSourceButton, &QPushButton::clicked, this, &CompressBagWidget::sourceButtonPressed);
    connect(targetLocationButton, &QPushButton::clicked, this, &CompressBagWidget::targetButtonPressed);
    connect(compressionFormatComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this] (int index) {
        writeParameterToSettings(m_parameters.compressPerMessage, index == 1, m_settings);
    });
    connect(deleteSourceCheckBox, &QCheckBox::stateChanged, this, [this] (int state) {
        writeParameterToSettings(m_parameters.deleteSource, state == Qt::Checked, m_settings);
    });
    connect(m_okButton, &QPushButton::clicked, this, &CompressBagWidget::okButtonPressed);

    setPixmapLabelIcon();
}


void
CompressBagWidget::sourceButtonPressed()
{
    const auto fileName = QFileDialog::getExistingDirectory(this, "Open Bag", "", QFileDialog::ShowDirsOnly);
    if (fileName.isEmpty()) {
        return;
    }

    writeParameterToSettings(m_parameters.sourceDirectory, fileName, m_settings);
    m_sourceLineEdit->setText(fileName);

    enableOkButton(!m_parameters.sourceDirectory.isEmpty() && !m_parameters.targetDirectory.isEmpty());
}


void
CompressBagWidget::targetButtonPressed()
{
    const auto fileName = QFileDialog::getSaveFileName(this, "Save Bag");
    if (fileName.isEmpty()) {
        return;
    }

    writeParameterToSettings(m_parameters.targetDirectory, fileName, m_settings);
    m_targetLineEdit->setText(fileName);
    enableOkButton(!m_parameters.sourceDirectory.isEmpty() && !m_parameters.targetDirectory.isEmpty());
}


void
CompressBagWidget::okButtonPressed()
{
    if (!Utils::ROS::doesDirectoryContainBagFile(m_parameters.sourceDirectory)) {
        Utils::UI::createCriticalMessageBox("Invalid bag file!", "The source bag file seems to be invalid or is already compressed.\n"
                                            "Please make sure that the bag file is valid and uncompressed!");
        return;
    }
    if (!Utils::UI::continueForExistingTarget(m_parameters.targetDirectory, "Bagfile", "bag file")) {
        return;
    }

    emit okPressed();
}
