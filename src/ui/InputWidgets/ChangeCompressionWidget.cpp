#include "ChangeCompressionWidget.hpp"

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

ChangeCompressionWidget::ChangeCompressionWidget(Parameters::CompressBagParameters& parameters, bool compress,
                                                 QWidget *parent) :
    BasicInputWidget(compress ? "Compress Bag" : "Decompress Bag",
                     compress ? ":/icons/compress_bag" : ":/icons/decompress_bag", parent),
    m_parameters(parameters), m_settings(parameters, compress ? "compress_bag" : "decompress_bag"), m_compress(compress)
{
    m_sourceLineEdit->setText(parameters.sourceDirectory);
    m_sourceLineEdit->setToolTip(compress ? "The source bag file directory." : "The compressed source bag file directory.");

    m_targetLineEdit = new QLineEdit(m_parameters.targetDirectory);
    m_targetLineEdit->setText(parameters.targetDirectory);
    m_targetLineEdit->setToolTip(compress ? "The target compressed bag file directory." : "The target uncompressed bag file directory.");

    auto* const targetLocationButton = new QToolButton;
    auto* const targetLocationLayout = Utils::UI::createLineEditButtonLayout(m_targetLineEdit, targetLocationButton);

    auto* const deleteSourceCheckBox = new QCheckBox;
    deleteSourceCheckBox->setCheckState(m_parameters.deleteSource ? Qt::Checked : Qt::Unchecked);

    auto* const formLayout = new QFormLayout;
    formLayout->addRow("Source Bag Location:", m_findSourceLayout);
    formLayout->addRow("Target Bag Location:", targetLocationLayout);
    formLayout->addRow("Delete Source Bag:", deleteSourceCheckBox);

    if (m_compress) {
        auto* const compressionFormatComboBox = new QComboBox;
        compressionFormatComboBox->addItem("By File");
        compressionFormatComboBox->addItem("Per Message");
        compressionFormatComboBox->setCurrentIndex(m_parameters.compressPerMessage);

        formLayout->insertRow(2, "Compression Mode:", compressionFormatComboBox);

        connect(compressionFormatComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this] (int index) {
            writeParameterToSettings(m_parameters.compressPerMessage, index == 1, m_settings);
        });
    }

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

    connect(m_findSourceButton, &QPushButton::clicked, this, &ChangeCompressionWidget::sourceButtonPressed);
    connect(targetLocationButton, &QPushButton::clicked, this, &ChangeCompressionWidget::targetButtonPressed);
    connect(deleteSourceCheckBox, &QCheckBox::stateChanged, this, [this] (int state) {
        writeParameterToSettings(m_parameters.deleteSource, state == Qt::Checked, m_settings);
    });
    connect(m_okButton, &QPushButton::clicked, this, &ChangeCompressionWidget::okButtonPressed);

    setPixmapLabelIcon();
}


void
ChangeCompressionWidget::sourceButtonPressed()
{
    const auto fileName = QFileDialog::getExistingDirectory(this, "Open Source Bag File", "", QFileDialog::ShowDirsOnly);
    if (fileName.isEmpty()) {
        return;
    }
    if (const auto valid = isBagFileValid(fileName); !valid) {
        return;
    }

    writeParameterToSettings(m_parameters.sourceDirectory, fileName, m_settings);
    m_sourceLineEdit->setText(fileName);

    enableOkButton(!m_parameters.sourceDirectory.isEmpty() && !m_parameters.targetDirectory.isEmpty());
}


void
ChangeCompressionWidget::targetButtonPressed()
{
    const auto fileName = QFileDialog::getSaveFileName(this, m_compress ? "Save Compressed Bag" : "Save Uncompressed Bag");
    if (fileName.isEmpty()) {
        return;
    }

    writeParameterToSettings(m_parameters.targetDirectory, fileName, m_settings);
    m_targetLineEdit->setText(fileName);
    enableOkButton(!m_parameters.sourceDirectory.isEmpty() && !m_parameters.targetDirectory.isEmpty());
}


void
ChangeCompressionWidget::okButtonPressed()
{
    if (const auto valid = isBagFileValid(m_parameters.sourceDirectory); !valid) {
        return;
    }
    if (!Utils::UI::continueForExistingTarget(m_parameters.targetDirectory, "Bagfile", "bag file")) {
        return;
    }

    emit okPressed();
}


bool
ChangeCompressionWidget::isBagFileValid(const QString& bagDirectory)
{
    if (m_compress) {
        if (!Utils::ROS::doesDirectoryContainBagFile(bagDirectory)) {
            Utils::UI::createCriticalMessageBox("Invalid bag file!", "The source bag file seems to be invalid or is already compressed.\n"
                                                "Please make sure that the bag file is valid and uncompressed!");
            return false;
        }
    } else if (!Utils::ROS::doesDirectoryContainCompressedBagFile(bagDirectory)) {
        Utils::UI::createCriticalMessageBox("Invalid bag file!", "The source bag file seems to be invalid or is not compressed.\n"
                                            "Please make sure that the bag file is valid and in compressed format!");
        return false;
    }

    return true;
}
