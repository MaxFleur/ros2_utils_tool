#include "ChangeCompressionWidget.hpp"

#include "LowDiskSpaceWidget.hpp"
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
#include <QShortcut>
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

    m_lowDiskSpaceWidget = new LowDiskSpaceWidget;

    auto* const formLayout = new QFormLayout;
    formLayout->addRow("Source Bag Location:", m_findSourceLayout);
    formLayout->addRow("Target Bag Location:", targetLocationLayout);
    formLayout->addRow("Delete Source Bag:", deleteSourceCheckBox);

    if (m_compress) {
        auto* const fileRadioButton = new QRadioButton("Compress File");
        fileRadioButton->setToolTip("Compresses the entire bag file. More efficient, but takes more time.");
        fileRadioButton->setChecked(!m_parameters.compressPerMessage);

        auto* const messageRadioButton = new QRadioButton("Compress per Message");
        messageRadioButton->setToolTip("Compresses each message individually. Faster, but less efficient.");
        messageRadioButton->setChecked(m_parameters.compressPerMessage);

        // Dummy space
        formLayout->addRow("", new QLabel(""));
        formLayout->addRow("Compression Mode:", fileRadioButton);
        formLayout->addRow("", messageRadioButton);

        connect(fileRadioButton, &QRadioButton::toggled, this, [this, messageRadioButton] (bool switched) {
            writeParameterToSettings(m_parameters.compressPerMessage, !switched, m_settings);
            messageRadioButton->setChecked(false);
        });
        connect(messageRadioButton, &QRadioButton::toggled, this, [this, fileRadioButton] (bool switched) {
            writeParameterToSettings(m_parameters.compressPerMessage, switched, m_settings);
            fileRadioButton->setChecked(false);
        });
    }

    auto* const controlsLayout = new QVBoxLayout;
    controlsLayout->addStretch();
    controlsLayout->addWidget(m_headerPixmapLabel);
    controlsLayout->addWidget(m_headerLabel);
    controlsLayout->addSpacing(40);
    controlsLayout->addLayout(formLayout);
    controlsLayout->addSpacing(5);
    controlsLayout->addWidget(m_lowDiskSpaceWidget);
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

    auto* const okShortCut = new QShortcut(QKeySequence(Qt::Key_Return), this);

    connect(m_findSourceButton, &QPushButton::clicked, this, &ChangeCompressionWidget::sourceButtonPressed);
    connect(targetLocationButton, &QPushButton::clicked, this, &ChangeCompressionWidget::targetButtonPressed);
    connect(deleteSourceCheckBox, &QCheckBox::stateChanged, this, [this] (int state) {
        writeParameterToSettings(m_parameters.deleteSource, state == Qt::Checked, m_settings);
    });
    connect(m_okButton, &QPushButton::clicked, this, &ChangeCompressionWidget::okButtonPressed);
    connect(okShortCut, &QShortcut::activated, this, &ChangeCompressionWidget::okButtonPressed);

    setPixmapLabelIcon();
    setLowDiskSpaceWidgetVisibility(m_targetLineEdit->text());
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
    setLowDiskSpaceWidgetVisibility(m_targetLineEdit->text());
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
