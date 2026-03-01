#include "ChangeCompressionWidget.hpp"

#include "UtilsROS.hpp"
#include "UtilsUI.hpp"

#include <QCheckBox>
#include <QFileDialog>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QRadioButton>
#include <QVBoxLayout>

ChangeCompressionWidget::ChangeCompressionWidget(Parameters::CompressBagParameters& parameters, bool compress,
                                                 QWidget *parent) :
    AdvancedInputWidget(parameters, compress ? "Compress Bag" : "Decompress Bag",
                        compress ? ":/icons/tools/compress_bag" : ":/icons/tools/decompress_bag", "Source Bag:", "Target Bag:",
                        compress ? "compress_bag" : "decompress_bag", compress ? OUTPUT_BAG_COMPRESSED : OUTPUT_BAG_DECOMPRESSED, parent),
    m_parameters(parameters), m_settings(parameters, compress ? "compress_bag" : "decompress_bag"), m_compress(compress)
{
    m_sourceLineEdit->setToolTip(compress ? "The source bag file directory." : "The compressed source bag file directory.");
    m_targetLineEdit->setToolTip(compress ? "The target compressed bag file directory." : "The target uncompressed bag file directory.");

    auto* const deleteSourceCheckBox = new QCheckBox;
    deleteSourceCheckBox->setCheckState(m_parameters.deleteSource ? Qt::Checked : Qt::Unchecked);

    m_basicOptionsFormLayout->addRow("Delete Source Bag:", deleteSourceCheckBox);

    if (m_compress) {
        auto* const fileRadioButton = new QRadioButton("Compress File");
        fileRadioButton->setToolTip("Compresses the entire bag file. More efficient, but takes more time.");
        fileRadioButton->setChecked(!m_parameters.compressPerMessage);

        auto* const messageRadioButton = new QRadioButton("Compress per Message");
        messageRadioButton->setToolTip("Compresses each message individually. Faster, but less efficient.");
        messageRadioButton->setChecked(m_parameters.compressPerMessage);

        // Dummy space
        m_basicOptionsFormLayout->addRow("", new QLabel(""));
        m_basicOptionsFormLayout->addRow("Compression Mode:", fileRadioButton);
        m_basicOptionsFormLayout->addRow("", messageRadioButton);

        connect(fileRadioButton, &QRadioButton::toggled, this, [this, messageRadioButton] (bool switched) {
            writeParameterToSettings(m_parameters.compressPerMessage, !switched, m_settings);
            messageRadioButton->setChecked(false);
        });
        connect(messageRadioButton, &QRadioButton::toggled, this, [this, fileRadioButton] (bool switched) {
            writeParameterToSettings(m_parameters.compressPerMessage, switched, m_settings);
            fileRadioButton->setChecked(false);
        });
    }

    m_controlsLayout->addStretch();
    enableOkButton(!m_parameters.sourceDirectory.isEmpty() && !m_parameters.targetDirectory.isEmpty());

    connect(deleteSourceCheckBox, &QCheckBox::stateChanged, this, [this] (int state) {
        writeParameterToSettings(m_parameters.deleteSource, state == Qt::Checked, m_settings);
    });
}


void
ChangeCompressionWidget::findSourceButtonPressed()
{
    const auto fileName = QFileDialog::getExistingDirectory(this, "Open Source Bag File", "", QFileDialog::ShowDirsOnly);
    if (fileName.isEmpty()) {
        return;
    }
    if (const auto valid = isBagFileValid(fileName); !valid) {
        return;
    }

    m_sourceLineEdit->setText(fileName);
    writeParameterToSettings(m_parameters.sourceDirectory, fileName, m_settings);
    fillTargetLineEdit();
    enableOkButton(!m_parameters.sourceDirectory.isEmpty() && !m_parameters.targetDirectory.isEmpty());
}


void
ChangeCompressionWidget::okButtonPressed() const
{
    if (const auto valid = isBagFileValid(m_parameters.sourceDirectory); !valid) {
        return;
    }
    if (const auto sufficientSpace = showLowDiskSpaceMessageBox(); !sufficientSpace) {
        return;
    }
    if (!Utils::UI::continueForExistingTarget(m_parameters.targetDirectory, "Bagfile", "bag file")) {
        return;
    }

    emit okPressed();
}


bool
ChangeCompressionWidget::isBagFileValid(const QString& bagDirectory) const
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
