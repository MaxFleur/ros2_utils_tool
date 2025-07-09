#include "SettingsDialog.hpp"

#include "BasicSettings.hpp"
#include "UtilsUI.hpp"

#include <QCheckBox>
#include <QDialogButtonBox>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QMessageBox>
#include <QSettings>
#include <QSpinBox>
#include <QVBoxLayout>

SettingsDialog::SettingsDialog(Parameters::DialogParameters& parameters, QWidget* parent) :
    QDialog(parent), m_parameters(parameters), m_settings(parameters, "dialog")
{
    setWindowTitle("Options");

    auto* const threadsLabel = new QLabel("Maximum Number of Threads:");

    auto* const maxNumberOfThreadsSpinBox = new QSpinBox;
    maxNumberOfThreadsSpinBox->setRange(1, std::thread::hardware_concurrency());
    maxNumberOfThreadsSpinBox->setToolTip("The maximum number of threads used for some tools.\n"
                                          "A higher number of threads will increase tool performance,\n"
                                          "but might make the system more laggy.");
    maxNumberOfThreadsSpinBox->setValue(m_parameters.maxNumberOfThreads);

    auto* const threadsLayout = new QHBoxLayout;
    threadsLayout->addWidget(threadsLabel);
    threadsLayout->addWidget(maxNumberOfThreadsSpinBox);

    auto* const useHardwareAccCheckBox = Utils::UI::createCheckBox("Use hardware acceleration for some tools.", m_parameters.useHardwareAcceleration);
    useHardwareAccCheckBox->setText("Use Hardware Acceleration");

    auto* const warnROS2NamesConventionCheckBox = Utils::UI::createCheckBox("If the tool should put out a warning\n"
                                                                            "if topic names are not following ROS2 conventions.",
                                                                            m_parameters.warnROS2NameConvention);
    warnROS2NamesConventionCheckBox->setText("Topic Names not following ROS2 Conventions");
    auto* const warnOverwriteTargetCheckBox = Utils::UI::createCheckBox("If the tool should ask to continue if a target file is overwritten.",
                                                                        m_parameters.warnTargetOverwrite);
    warnOverwriteTargetCheckBox->setText("Overwriting exisiting Target Files");
    auto* const warnLowDiskspaceCheckBox = Utils::UI::createCheckBox("If the tool should put out a warning for low disk space.",
                                                                     m_parameters.warnLowDiskSpace);
    warnLowDiskspaceCheckBox->setText("Low available Disk Space");

    auto* const showWarningsLabel = new QLabel("Show Warning Message Boxes for...");

    auto* const storeParametersCheckBox = Utils::UI::createCheckBox("If this is checked, all input parameters are saved\n"
                                                                    "and reused if this application is launched another time.",
                                                                    m_parameters.saveParameters);
    storeParametersCheckBox->setText("Save Input Parameters");
    auto* const usePredefinedTopicNamesCheckBox = Utils::UI::createCheckBox("Use some optional predefined topic names for the publishing and video to bag tools.",
                                                                            m_parameters.usePredefinedTopicNames);
    usePredefinedTopicNamesCheckBox->setText("Use Predefined Topic Names");

    auto* const buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);

    auto* const systemLayout = new QVBoxLayout;
    systemLayout->addLayout(threadsLayout);
    systemLayout->addWidget(useHardwareAccCheckBox);

    auto* const systemGroupBox = new QGroupBox("System");
    systemGroupBox->setLayout(systemLayout);

    auto* const warnLayout = new QVBoxLayout;
    warnLayout->addWidget(showWarningsLabel, Qt::AlignLeft);
    warnLayout->addWidget(warnROS2NamesConventionCheckBox);
    warnLayout->addWidget(warnOverwriteTargetCheckBox);
    warnLayout->addWidget(warnLowDiskspaceCheckBox);

    auto* const warnGroupBox = new QGroupBox("Warnings");
    warnGroupBox->setLayout(warnLayout);

    auto* const miscLayout = new QVBoxLayout;
    miscLayout->addWidget(storeParametersCheckBox);
    miscLayout->addWidget(usePredefinedTopicNamesCheckBox);

    auto* const miscGroupBox = new QGroupBox("Miscellaneous");
    miscGroupBox->setLayout(miscLayout);

    // Set main layout
    auto* const mainLayout = new QVBoxLayout(this);
    mainLayout->addWidget(systemGroupBox);
    mainLayout->addSpacing(10);
    mainLayout->addWidget(warnGroupBox);
    mainLayout->addSpacing(10);
    mainLayout->addWidget(miscGroupBox);
    mainLayout->addWidget(buttonBox);
    setLayout(mainLayout);

    connect(buttonBox, &QDialogButtonBox::accepted, this, [this, maxNumberOfThreadsSpinBox, useHardwareAccCheckBox,
                                                           storeParametersCheckBox, usePredefinedTopicNamesCheckBox,
                                                           warnROS2NamesConventionCheckBox, warnOverwriteTargetCheckBox,
                                                           warnLowDiskspaceCheckBox] {
        m_parameters.maxNumberOfThreads = maxNumberOfThreadsSpinBox->value();
        m_parameters.useHardwareAcceleration = useHardwareAccCheckBox->checkState() == Qt::Checked;
        m_parameters.saveParameters = storeParametersCheckBox->checkState() == Qt::Checked;
        m_parameters.usePredefinedTopicNames = usePredefinedTopicNamesCheckBox->checkState() == Qt::Checked;
        m_parameters.warnROS2NameConvention = warnROS2NamesConventionCheckBox->checkState() == Qt::Checked;
        m_parameters.warnTargetOverwrite = warnOverwriteTargetCheckBox->checkState() == Qt::Checked;
        m_parameters.warnLowDiskSpace = warnLowDiskspaceCheckBox->checkState() == Qt::Checked;
        m_settings.write();
        QDialog::accept();
    });
    connect(buttonBox, &QDialogButtonBox::rejected, this, [this] {
        QDialog::reject();
    });
}


// Need to restart for effects to take place
void
SettingsDialog::storeParametersCheckStateChanged()
{
    auto* const msgBox = new QMessageBox();
    msgBox->setIcon(QMessageBox::Information);
    msgBox->setText("Changes will take effect after restarting the application.");
    msgBox->exec();
}
