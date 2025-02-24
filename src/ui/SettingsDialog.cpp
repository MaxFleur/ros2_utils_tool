#include "SettingsDialog.hpp"

#include "BasicSettings.hpp"

#include <QCheckBox>
#include <QDialogButtonBox>
#include <QMessageBox>
#include <QSettings>
#include <QVBoxLayout>

SettingsDialog::SettingsDialog(Parameters::DialogParameters& dialogParameters, QWidget* parent) :
    QDialog(parent), m_dialogSettings(dialogParameters, "dialog"), m_dialogParameters(dialogParameters)
{
    setWindowTitle("Options");

    m_storeParametersCheckBox = new QCheckBox("Save Input Parameters");
    m_storeParametersCheckBox->setTristate(false);
    m_storeParametersCheckBox->setToolTip("If this is checked, all input parameters are saved\n"
                                          "and reused if this application is launched another time.");
    m_storeParametersCheckBox->setCheckState(m_dialogParameters.saveParameters ? Qt::Checked : Qt::Unchecked);

    m_usePredefinedTopicNamesCheckBox = new QCheckBox("Use Predefined Topic Names");
    m_usePredefinedTopicNamesCheckBox->setTristate(false);
    m_usePredefinedTopicNamesCheckBox->setToolTip("Use some optional predefined topic names for the publishing and video to bag tools.");
    m_usePredefinedTopicNamesCheckBox->setCheckState(m_dialogParameters.usePredefinedTopicNames ? Qt::Checked : Qt::Unchecked);

    m_checkROS2NamingConventionCheckBox = new QCheckBox("Check for ROS2 Naming Conventions");
    m_checkROS2NamingConventionCheckBox->setTristate(false);
    m_checkROS2NamingConventionCheckBox->setToolTip("If input fields requiring topic names should check\nfor ROS2 Topic Naming Conventions.");
    m_checkROS2NamingConventionCheckBox->setCheckState(m_dialogParameters.checkROS2NameConform ? Qt::Checked : Qt::Unchecked);

    auto* const buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);

    connect(m_storeParametersCheckBox, &QCheckBox::stateChanged, this, &SettingsDialog::storeParametersCheckStateChanged);
    connect(buttonBox, &QDialogButtonBox::accepted, this, &SettingsDialog::okClicked);
    connect(buttonBox, &QDialogButtonBox::rejected, this, [this] {
        QDialog::reject();
    });

    // Set main layout
    auto* const mainLayout = new QVBoxLayout(this);
    mainLayout->addWidget(m_storeParametersCheckBox);
    mainLayout->addWidget(m_usePredefinedTopicNamesCheckBox);
    mainLayout->addWidget(m_checkROS2NamingConventionCheckBox);
    mainLayout->addWidget(buttonBox);
    setLayout(mainLayout);
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


void
SettingsDialog::okClicked()
{
    m_dialogParameters.saveParameters = m_storeParametersCheckBox->checkState() == Qt::Checked;
    m_dialogParameters.usePredefinedTopicNames = m_usePredefinedTopicNamesCheckBox->checkState() == Qt::Checked;
    m_dialogParameters.checkROS2NameConform = m_checkROS2NamingConventionCheckBox->checkState() == Qt::Checked;
    m_dialogSettings.write();
    QDialog::accept();
}
