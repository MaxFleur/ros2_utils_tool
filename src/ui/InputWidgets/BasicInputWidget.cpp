#include "BasicInputWidget.hpp"

#include "UtilsUI.hpp"

#include <QDialogButtonBox>
#include <QEvent>
#include <QHBoxLayout>
#include <QLabel>
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


bool
BasicInputWidget::event(QEvent *event)
{
    [[unlikely]] if (event->type() == QEvent::ApplicationPaletteChange || event->type() == QEvent::PaletteChange) {
        setPixmapLabelIcon();
    }
    return QWidget::event(event);
}
