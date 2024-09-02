#include "PlayBagConfigWidget.hpp"

#include "UtilsROS.hpp"

#include <QCheckBox>
#include <QDialogButtonBox>
#include <QFileDialog>
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

PlayBagConfigWidget::PlayBagConfigWidget(Utils::UI::PlayBagParameters& bagParameters, QWidget *parent) :
    BasicConfigWidget(":/icons/play_bag_white.svg", ":/icons/play_bag_black.svg", parent),
    m_bagParameters(bagParameters)
{
    auto* const headerTextLabel = new QLabel("Play ROSBag");
    Utils::UI::setWidgetHeaderFont(headerTextLabel);
    headerTextLabel->setAlignment(Qt::AlignHCenter);

    m_bagLineEdit = new QLineEdit(m_bagParameters.bagDirectory);
    m_bagLineEdit->setToolTip("The directory of the ROSBag.");
    auto* const bagLocationButton = new QToolButton;
    auto* const bagLayout = Utils::UI::createLineEditButtonLayout(m_bagLineEdit, bagLocationButton);

    auto* const loopCheckBox = new QCheckBox;
    loopCheckBox->setToolTip("Whether to loop the bag file.");
    loopCheckBox->setCheckState(m_bagParameters.loop ? Qt::Checked : Qt::Unchecked);

    auto* const formLayout = new QFormLayout;
    formLayout->addRow("Bag Location:", bagLayout);
    formLayout->addRow("Loop Bag File:", loopCheckBox);

    auto* const controlsLayout = new QVBoxLayout;
    controlsLayout->addStretch();
    controlsLayout->addWidget(m_headerPixmapLabel);
    controlsLayout->addWidget(headerTextLabel);
    controlsLayout->addSpacing(40);
    controlsLayout->addLayout(formLayout);
    controlsLayout->addStretch();

    auto* const controlsSqueezedLayout = new QHBoxLayout;
    controlsSqueezedLayout->addStretch();
    controlsSqueezedLayout->addLayout(controlsLayout);
    controlsSqueezedLayout->addStretch();

    auto* const backButton = new QPushButton("Back");

    auto* const buttonBox = new QDialogButtonBox;
    buttonBox->addButton(m_okButton, QDialogButtonBox::AcceptRole);

    auto* const buttonLayout = new QHBoxLayout;
    buttonLayout->addWidget(backButton);
    buttonLayout->addStretch();
    buttonLayout->addWidget(buttonBox);

    auto* const mainLayout = new QVBoxLayout;
    mainLayout->addLayout(controlsSqueezedLayout);
    mainLayout->addLayout(buttonLayout);
    setLayout(mainLayout);

    auto* const okShortCut = new QShortcut(QKeySequence(Qt::Key_Return), this);
    enableOkButton(!m_bagLineEdit->text().isEmpty());

    connect(bagLocationButton, &QPushButton::clicked, this, &PlayBagConfigWidget::bagLocationButtonPressed);
    connect(loopCheckBox, &QCheckBox::stateChanged, this, [this] (int state) {
        m_bagParameters.loop = state == Qt::Checked;
    });
    connect(backButton, &QPushButton::clicked, this, [this] {
        emit back();
    });
    connect(buttonBox, &QDialogButtonBox::accepted, this, &PlayBagConfigWidget::okButtonPressed);
    connect(okShortCut, &QShortcut::activated, this, &PlayBagConfigWidget::okButtonPressed);
}


void
PlayBagConfigWidget::bagLocationButtonPressed()
{
    const auto bagDirectory = QFileDialog::getExistingDirectory(this, "Open Directory", "",
                                                                QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
    if (bagDirectory.isEmpty()) {
        return;
    }

    m_bagParameters.bagDirectory = bagDirectory;
    m_bagLineEdit->setText(bagDirectory);
    enableOkButton(!m_bagLineEdit->text().isEmpty());
}


void
PlayBagConfigWidget::okButtonPressed()
{
    if (!m_okButton->isEnabled()) {
        return;
    }

#ifdef ROS_HUMBLE
    auto *const msgBox = new QMessageBox(QMessageBox::Warning, "Continue?",
                                         "Currently, you are using ROS Humble. Due to API restrictions, the application has to be shut down "
                                         "if you stop playing the bag file. Do you want to continue?",
                                         QMessageBox::Yes | QMessageBox::No);
    const auto ret = msgBox->exec();
    if (ret == QMessageBox::No) {
        return;
    }
#endif

    emit okPressed();
}
