#include "SettingsDialog.hpp"

#include "BasicSettings.hpp"

#include <QCheckBox>
#include <QDialogButtonBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QMessageBox>
#include <QSettings>
#include <QSpinBox>
#include <QVBoxLayout>

SettingsDialog::SettingsDialog(Parameters::DialogParameters& dialogParameters, QWidget* parent) :
    QDialog(parent), m_dialogSettings(dialogParameters, "dialog"), m_dialogParameters(dialogParameters)
{
    setWindowTitle("Options");

    auto* const threadsLabel = new QLabel("Maximum Number of Threads:");

    auto* const maxNumberOfThreadsSpinBox = new QSpinBox;
    maxNumberOfThreadsSpinBox->setRange(1, std::thread::hardware_concurrency());
    maxNumberOfThreadsSpinBox->setToolTip("The maximum number of threads used for some tools.\n"
                                          "A higher number of threads will increase tool performance,\n"
                                          "but might make the system more laggy.");
    maxNumberOfThreadsSpinBox->setValue(m_dialogParameters.maxNumberOfThreads);

    auto* const threadsLayout = new QHBoxLayout;
    threadsLayout->addWidget(threadsLabel);
    threadsLayout->addWidget(maxNumberOfThreadsSpinBox);

    auto* const storeParametersCheckBox = new QCheckBox("Save Input Parameters");
    storeParametersCheckBox->setTristate(false);
    storeParametersCheckBox->setToolTip("If this is checked, all input parameters are saved\n"
                                        "and reused if this application is launched another time.");
    storeParametersCheckBox->setCheckState(m_dialogParameters.saveParameters ? Qt::Checked : Qt::Unchecked);

    auto* const usePredefinedTopicNamesCheckBox = new QCheckBox("Use Predefined Topic Names");
    usePredefinedTopicNamesCheckBox->setTristate(false);
    usePredefinedTopicNamesCheckBox->setToolTip("Use some optional predefined topic names for the publishing and video to bag tools.");
    usePredefinedTopicNamesCheckBox->setCheckState(m_dialogParameters.usePredefinedTopicNames ? Qt::Checked : Qt::Unchecked);

    auto* const checkROS2NamingConventionCheckBox = new QCheckBox("Check for ROS2 Naming Conventions");
    checkROS2NamingConventionCheckBox->setTristate(false);
    checkROS2NamingConventionCheckBox->setToolTip("If input fields requiring topic names should check\nfor ROS2 Topic Naming Conventions.");
    checkROS2NamingConventionCheckBox->setCheckState(m_dialogParameters.checkROS2NameConform ? Qt::Checked : Qt::Unchecked);

    auto* const buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);

    // Set main layout
    auto* const mainLayout = new QVBoxLayout(this);
    mainLayout->addLayout(threadsLayout);
    mainLayout->addSpacing(10);
    mainLayout->addWidget(storeParametersCheckBox);
    mainLayout->addWidget(usePredefinedTopicNamesCheckBox);
    mainLayout->addWidget(checkROS2NamingConventionCheckBox);
    mainLayout->addWidget(buttonBox);
    setLayout(mainLayout);

    connect(buttonBox, &QDialogButtonBox::accepted, this, [this, maxNumberOfThreadsSpinBox, storeParametersCheckBox,
                                                           usePredefinedTopicNamesCheckBox, checkROS2NamingConventionCheckBox] {
        m_dialogParameters.maxNumberOfThreads = maxNumberOfThreadsSpinBox->value();
        m_dialogParameters.saveParameters = storeParametersCheckBox->checkState() == Qt::Checked;
        m_dialogParameters.usePredefinedTopicNames = usePredefinedTopicNamesCheckBox->checkState() == Qt::Checked;
        m_dialogParameters.checkROS2NameConform = checkROS2NamingConventionCheckBox->checkState() == Qt::Checked;
        m_dialogSettings.write();
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
