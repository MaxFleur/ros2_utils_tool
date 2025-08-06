#include "BasicInputWidget.hpp"

#include "DialogSettings.hpp"
#include "LowDiskSpaceWidget.hpp"
#include "UtilsGeneral.hpp"
#include "UtilsUI.hpp"

#include <QDialogButtonBox>
#include <QDir>
#include <QEvent>
#include <QHBoxLayout>
#include <QLabel>
#include <QMessageBox>
#include <QPushButton>
#include <QToolButton>

#include <filesystem>

BasicInputWidget::BasicInputWidget(const QString& headerText, const QString& iconPath, QWidget *parent) :
    QWidget(parent), m_iconPath(iconPath)
{
    m_headerLabel = new QLabel(headerText);
    Utils::UI::setWidgetFontSize(m_headerLabel);
    m_headerLabel->setAlignment(Qt::AlignHCenter);

    m_headerPixmapLabel = new QLabel;
    m_headerPixmapLabel->setAlignment(Qt::AlignHCenter);
    setPixmapLabelIcon();

    m_sourceLineEdit = new QLineEdit;
    m_findSourceButton = new QToolButton;

    m_backButton = new QPushButton("Back", this);
    m_okButton = new QPushButton("Ok", this);
    m_okButton->setEnabled(false);

    m_dialogButtonBox = new QDialogButtonBox;
    m_dialogButtonBox->addButton(m_okButton, QDialogButtonBox::AcceptRole);
    // Layout can be already complete
    m_findSourceLayout = Utils::UI::createLineEditButtonLayout(m_sourceLineEdit, m_findSourceButton);

    m_buttonLayout = new QHBoxLayout;
    m_buttonLayout->addWidget(m_backButton);
    m_buttonLayout->addStretch();
    m_buttonLayout->addWidget(m_dialogButtonBox);

    connect(m_backButton, &QPushButton::clicked, this, [this] {
        emit back();
    });
}


void
BasicInputWidget::enableOkButton(bool enable) const
{
    m_okButton->setEnabled(enable);
}


void
BasicInputWidget::setPixmapLabelIcon() const
{
    const auto isDarkMode = Utils::UI::isDarkMode();
    // Don't need to provide full name, appendix is always the same
    m_headerPixmapLabel->setPixmap(QIcon(isDarkMode ? m_iconPath + "_white.svg" : m_iconPath + "_black.svg").pixmap(QSize(100, 45)));
}


void
BasicInputWidget::setLowDiskSpaceWidgetVisibility(const QString& path)
{
    if (path.isEmpty()) {
        return;
    }

    QDir dir(path);
    dir.cdUp();

    m_lowDiskSpaceWidget->setVisibility(dir.path());
    m_remainingSpace = Utils::General::getAvailableDriveSpace(dir.path());
}


bool
BasicInputWidget::showLowDiskSpaceMessageBox() const
{
    if (const auto sufficientSpace = m_lowDiskSpaceWidget->isDiskSpaceSpaceSufficient();
        !sufficientSpace && DialogSettings::getStaticParameter("warn_low_disk_space", true)) {
        auto *const msgBox = new QMessageBox(QMessageBox::Warning, "Small Disk Space!",
                                             "The available disk space is very small (" + QString::number(m_remainingSpace) + " GiB). Are you sure you want to continue? ",
                                             QMessageBox::Yes | QMessageBox::No);

        auto* const checkBox = Utils::UI::createMessageBoxCheckBox("warn_low_disk_space");
        msgBox->setCheckBox(checkBox);

        if (const auto ret = msgBox->exec(); ret == QMessageBox::No) {
            return false;
        }
    }
    return true;
}


bool
BasicInputWidget::event(QEvent *event)
{
    [[unlikely]] if (event->type() == QEvent::ApplicationPaletteChange || event->type() == QEvent::PaletteChange) {
        setPixmapLabelIcon();
    }
    return QWidget::event(event);
}
