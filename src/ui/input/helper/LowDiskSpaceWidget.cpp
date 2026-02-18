#include "LowDiskSpaceWidget.hpp"

#include "UtilsGeneral.hpp"

#include <QDir>
#include <QEvent>
#include <QHBoxLayout>
#include <QIcon>
#include <QLabel>

LowDiskSpaceWidget::LowDiskSpaceWidget(QWidget *parent) :
    QWidget(parent)
{
    m_warningIconLabel = new QLabel;
    m_diskSpaceLabel = new QLabel;

    auto labelFont = m_diskSpaceLabel->font();
    labelFont.setBold(true);
    m_diskSpaceLabel->setFont(labelFont);

    auto palette = m_diskSpaceLabel->palette();
    palette.setColor(QPalette::WindowText, Qt::red);
    m_diskSpaceLabel->setPalette(palette);

    auto* const mainLayout = new QHBoxLayout;
    mainLayout->addWidget(m_warningIconLabel);
    mainLayout->addWidget(m_diskSpaceLabel);
    mainLayout->addStretch();
    // Will be integrated into other widgets, so remove the extra space
    mainLayout->setContentsMargins(0, 0, 0, 0);
    setLayout(mainLayout);

    setPixmapLabelIcon();
}


void
LowDiskSpaceWidget::setVisibility(const QString& path)
{
    const auto diskSpace = Utils::General::getAvailableDriveSpace(path);
    m_isDiskSpaceSufficient = diskSpace > Utils::General::MINIMUM_RECOMMENDED_DRIVE_SPACE;

    if (!m_isDiskSpaceSufficient) {
        m_diskSpaceLabel->setText("Free available space is only " + QString::number(diskSpace) + " GiB!");
    }
    m_warningIconLabel->setVisible(!m_isDiskSpaceSufficient);
    m_diskSpaceLabel->setVisible(!m_isDiskSpaceSufficient);
}


void
LowDiskSpaceWidget::setPixmapLabelIcon() const
{
    const auto visibility = m_warningIconLabel->isVisible();

    m_warningIconLabel->setPixmap(QIcon(":/icons/widgets/warning.svg").pixmap(QSize(ICON_SIZE, ICON_SIZE)));
    m_warningIconLabel->setVisible(visibility);
}


bool
LowDiskSpaceWidget::event(QEvent *event)
{
    [[unlikely]] if (event->type() == QEvent::ApplicationPaletteChange || event->type() == QEvent::PaletteChange) {
        setPixmapLabelIcon();
    }
    return QWidget::event(event);
}
